#pragma once
#include <opencv2/opencv.hpp>
#include <time.h>

class RgbFrame{
public:
    RgbFrame():empty_frame(true){}
    RgbFrame(const double t, const cv::Mat m): frame(m), time_stamp(t), empty_frame(false){}
    // RgbFrame(const RgbFrame& rgb_frame): frame(rgb_frame.frame), time_stamp(rgb_frame.time_stamp), empty_frame(false){}
    // RgbFrame(RgbFrame&& rgb_frame): frame(rgb_frame.frame), time_stamp(std::move(rgb_frame.time_stamp)), empty_frame(rgb_frame.empty_frame){}
    cv::Mat frame;
    double time_stamp;
    bool empty_frame;    
};