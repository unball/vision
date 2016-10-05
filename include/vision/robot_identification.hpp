#ifndef IDENTIFICATION_H
#define IDENTIFICATION_H

#include <ros/ros.h>

#include <opencv2/opencv.hpp>
#include <vision/algorithm_factory.hpp>
#include <vision/identification_algorithm.hpp>
#include <vision/raw_image.hpp>

class RobotIdentification : public IdentificationAlgorithm
{
public:
    ALGORITHM_TYPE(RobotIdentification);
    void run();
    void init();

private:
    REGISTER_ALGORITHM_DEC(RobotIdentification);
    void find(cv::Mat input);
    void identify(cv::Mat rgb_input);
    std::vector<cv::Rect> robots_;

    cv::Mat segmentedImage_;
    int hsv_min_h_, hsv_max_h_;
    int hsv_min_s_, hsv_max_s_;
    int hsv_min_v_, hsv_max_v_;
    std::string window_name_ = "HSV";

    cv::Mat rgb_input_;
    cv::Mat depth_input_;
    std::vector<std::vector<cv::Point>> contours_;
    std::vector<cv::Vec4i> hierarchy_;

    int area_;
    bool hasclosed_; 

};
#endif