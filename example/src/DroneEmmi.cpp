






//--- baslangic


#include "DroneEmmi.h"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "tello.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <atomic>
#include <csignal>
#include <deque>

using namespace cv;
using namespace std;

BackgroundFrameRead::BackgroundFrameRead(const string& address, bool with_queue, int maxsize)
    : address(address), with_queue(with_queue), maxsize(maxsize), stopped(false) {
    capture.open(address);
    if (!capture.isOpened()) {
        throw runtime_error("Failed to open video stream");
    }
}

BackgroundFrameRead::~BackgroundFrameRead() {
    stop();
}

void BackgroundFrameRead::start() {
    worker = thread(&BackgroundFrameRead::update_frame, this);
}

void BackgroundFrameRead::stop() {
    stopped = true;
    if (worker.joinable()) {
        worker.join();
    }
}

void BackgroundFrameRead::update_frame() {
    Mat img;
    while (!stopped) {
        capture >> img;
        if (img.empty()) {
            continue;
        }

        lock_guard<mutex> guard(lock);
        if (with_queue) {
            if (frames.size() >= maxsize) {
                frames.pop_front();
            }
            frames.push_back(img.clone());
        } else {
            frame = img.clone();
        }
    }
}

Mat BackgroundFrameRead::get_frame() {
    lock_guard<mutex> guard(lock);
    return frame.clone();
}

Mat BackgroundFrameRead::get_queued_frame() {
    lock_guard<mutex> guard(lock);
    if (frames.empty()) {
        return Mat();
    }
    Mat img = frames.front();
    frames.pop_front();
    return img;
}

DroneEmmi::DroneEmmi() : frame_reader(nullptr) {
    // Initialize values
    hsvVals = {0, 0, 133, 179, 41, 242};
    sensors = 3;
    threshold = 0.2;
    width = 480;
    height = 360;
    sensitivity = 3;
    weights = {-25, -15, 0, 15, 25};
    fSpeed = 7;
    curve = 0;
    total_lr = 0;
    i = 0;
}

DroneEmmi::~DroneEmmi() {
    if (frame_reader) {
        frame_reader->stop();
        delete frame_reader;
    }
}

int clip(int n, int lower, int upper) {
  return std::max(lower, std::min(n, upper));
}

void printVector(const std::vector<int>& vec) {
    std::cout << "[ ";
    for (const auto& elem : vec) {
        std::cout << elem << " ";
    }
    std::cout << "]" << std::endl;
}



void signalHandler(int signal) {
    std::cout << "Ctrl+C received. Exiting..." << std::endl;
    // Perform cleanup or other actions here if needed
    std::exit(signal);
}

void DroneEmmi::sendCommands(const vector<int>& senOut, int cx) {
    int lr = (cx - width / 2) / sensitivity;
    lr = clip(lr, -10, 10);


    if (lr > -2 && lr < 2) lr = 0;

    int curve = weights[2]; // Default curve value

    cout << "senOut=";
    for (int i = 0; i < senOut.size(); ++i) {
        cout << senOut[i] << " ";
    }
    cout << endl;

    if (senOut == vector<int>{1, 0, 0}) {
        curve = weights[0];
    } else if (senOut == vector<int>{1, 1, 0}) {
        curve = weights[1];
    } else if (senOut == vector<int>{0, 1, 0}) {
        curve = weights[2];
    } else if (senOut == vector<int>{0, 1, 1}) {
        curve = weights[3];
    } else if (senOut == vector<int>{0, 0, 1}) {
        curve = weights[4];
    } else if (senOut == vector<int>{0, 0, 0}) {
        curve = weights[2];
    } else if (senOut == vector<int>{1, 1, 1}) {
        curve = weights[2];
    } else if (senOut == vector<int>{1, 0, 1}) {
        curve = weights[2];
    }

    tello.move(lr, fSpeed, 0, curve);
    //tello.move(0,0,0,0);
    this_thread::sleep_for(chrono::milliseconds(200));
}


Mat DroneEmmi::thresholding(const Mat& img) {
    
    Mat hsv;
    cvtColor(img, hsv, COLOR_RGB2HSV);

    Scalar lower(hsvVals[0], hsvVals[1], hsvVals[2]);
    Scalar upper(hsvVals[3], hsvVals[4], hsvVals[5]);

    Mat mask;
    inRange(hsv, lower, upper, mask);

    return mask;
}

int DroneEmmi::getContours(const Mat& imgThres, Mat& img) {
    int cx = 0;

    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;

    // Find contours
    findContours(imgThres, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    if (!contours.empty()) {
        // Find the biggest contour
        vector<Point> biggest = *max_element(contours.begin(), contours.end(), 
            [](const vector<Point>& a, const vector<Point>& b) {
                return contourArea(a) < contourArea(b);
            });

        // Get bounding rectangle
        Rect rect = boundingRect(biggest);

        int x = rect.x;
        int y = rect.y;
        int w = rect.width;
        int h = rect.height;

        // Calculate center
        cx = x + w / 2;
        int cy = y + h / 2;

        // Draw the biggest contour
        vector<vector<Point>> biggestContour = {biggest};
        drawContours(img, biggestContour, -1, Scalar(255, 0, 255), 7);

        // Draw center
        circle(img, Point(cx, cy), 10, Scalar(0, 255, 0), cv::FILLED);
    }

    return cx;
}



vector<int> DroneEmmi::getSensorOutput(const Mat& imgThres, int sensors){// vector yap bu part kisimlarini.



    int widthForParts = width / sensors;
    cv::Mat part1 = imgThres(cv::Rect(0, 0, widthForParts, imgThres.rows));
    cv::Mat part2 = imgThres(cv::Rect(widthForParts, 0, widthForParts, imgThres.rows));
    cv::Mat part3 = imgThres(cv::Rect(2 * widthForParts, 0, widthForParts, imgThres.rows));




    int totalPixels = (imgThres.cols / sensors) * imgThres.rows;
    cout<<totalPixels<<endl;

    vector<int> senOut;
    int pixelCount;

    Mat tempImg;
    for(int j = 0; j < sensors; j++){
        if(j == 0) tempImg = part1;
        else if(j == 1) tempImg = part2;
        else if(j == 2) tempImg = part3;

        pixelCount = countNonZero(tempImg);

        if (pixelCount > threshold * totalPixels){
            senOut.push_back(1);
        }
        else{
            senOut.push_back(0);
        }

        cv::imshow(to_string(j), tempImg);

    }

    cout << "Sensor Outputs: ";
    for (int i = 0; i < senOut.size(); ++i) {
        cout << senOut[i] << " ";
    }
    cout << endl;


    return senOut;
}

void DroneEmmi::start() {
    signal(SIGINT, signalHandler);
    tello.connect();
    tello.enable_video_stream();
    frame_reader = new BackgroundFrameRead("udp://0.0.0.0:11111");
    frame_reader->start();

    Mat imgThres;
    int cx;
    vector<int> senOut;

    while (true) {
        Mat img = frame_reader->get_frame();

        if (i == 0) {
            tello.takeoff();
            i += 1;
        }

        if (img.empty()) {
            continue;
        }

        cv::resize(img, img, cv::Size(width, height));
        cv::flip(img, img, 0);

        imgThres = thresholding(img);

        cx = getContours(imgThres, img);

        senOut = getSensorOutput(imgThres, sensors);
        sendCommands(senOut, cx);
        cv::imshow("Output", img);
        cv::imshow("Path", imgThres);
        if (cv::waitKey(1) == 27) {
            break;
        }
    }
}

void DroneEmmi::test() {
    tello.connect();
    tello.enable_video_stream();
    cv::VideoCapture capture{"udp://0.0.0.0:11111", cv::CAP_FFMPEG};

    int i = 0;

    tello.takeoff();
    while (true) {
        i++;
        cv::Mat frame;
        capture >> frame;
        if (!frame.empty()) {
            cv::imshow("Tello Stream", frame);
        }
        if (cv::waitKey(1) == 27) {
            break;
        }

        tello.move(0,10,0,0);
    }

    tello.land();
}

// Implement other member functions such as thresholding, getContours, getSensorOutput, and sendCommands here.




