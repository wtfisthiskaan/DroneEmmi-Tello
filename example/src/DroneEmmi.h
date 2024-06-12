#ifndef TELLO_DRONEEMMI_H
#define TELLO_DRONEEMMI_H
#include "tello.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <thread>
#include <atomic>
#include <mutex>
#include <deque>

using namespace std;
using namespace cv;

class BackgroundFrameRead {
    private:
        string address;
        thread worker;
        atomic<bool> stopped;
        mutex lock;
        Mat frame;
        deque<Mat> frames;
        bool with_queue;
        int maxsize;
        VideoCapture capture;

        void update_frame();

    public:
        BackgroundFrameRead(const string& address, bool with_queue = false, int maxsize = 32);
        ~BackgroundFrameRead();
        void start();
        void stop();
        Mat get_frame();
        Mat get_queued_frame();
};

class DroneEmmi {
    private:
        Tello tello;
        vector<int> hsvVals;
        int sensors;
        double threshold;
        int width, height;
        int sensitivity;
        vector<int> weights;
        int fSpeed;
        int curve;
        int total_lr;
        int i;
        Mat thresholding(const Mat& img);
        int getContours(const Mat& imgThres, Mat& img);
        vector<int> getSensorOutput(const Mat& imgThres, int sensors);
        void sendCommands(const vector<int>& senOut, int cx);
        void calibrate();

        BackgroundFrameRead* frame_reader;

    public:
        DroneEmmi();
        ~DroneEmmi();
        void start();
        void test();
};

#endif //TELLO_DRONEEMMI_H
