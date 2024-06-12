// #include "tello.hpp"

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "DroneEmmi.h"
#include "opencv2/imgproc/imgproc.hpp"

using namespace std;
using namespace cv;


int main() {



    DroneEmmi emmi;




    //emmi.test();
    emmi.start();


    //emmi.calibrate();

    //cv::VideoCapture capture(1);
    //cv::VideoCapture cap(0,cv::CAP_AVFOUNDATION);

}

// #include "tello.hpp"

// #include "opencv2/core.hpp"
// #include "opencv2/highgui.hpp"
// #include "opencv2/imgcodecs.hpp"
// #include "opencv2/imgproc/imgproc.hpp"

// using namespace cv;

// Mat thresholding(const Mat& img) {

//     std::vector<int> hsvVals;
//     hsvVals = {0,0,133,179,41,242}; // Initialize hsvVals to all zeros

    
//     Mat hsv;
//     cvtColor(img, hsv, COLOR_RGB2HSV);

//     Scalar lower(hsvVals[0], hsvVals[1], hsvVals[2]);
//     Scalar upper(hsvVals[3], hsvVals[4], hsvVals[5]);

//     Mat mask;
//     inRange(hsv, lower, upper, mask);

//     return mask;
// }

// int main() {

//     cv::VideoCapture capture{"udp://0.0.0.0:11111", cv::CAP_FFMPEG}; // CAP_FFMPEG

//     Tello tello;
//     if (!tello.connect()) return 0;
//     tello.enable_video_stream();

//     while (true) {
//         cv::Mat frame;
//         cv::Mat imgThres;

//         capture >> frame;
//         if (!frame.empty()) {
//             cv::resize(frame, frame, cv::Size(480, 360));
//             cv::flip(frame, frame, 0);

//             imgThres = thresholding(frame);


//             cv::imshow("Tello Stream", frame);
//             cv::imshow("Theresholded", imgThres);


//         }
//         if (cv::waitKey(1) == 27) {
//             break;
//         }
//     }
// }
