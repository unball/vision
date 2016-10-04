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
    void indentify(cv::Mat input);
    std::vector<cv::Rect> robots_;
    cv::Mat rgb_input_;
    cv::Mat depth_input_;
    std::vector<std::vector<cv::Point>> contours_;
    std::vector<cv::Vec4i> hierarchy_;

    int area_;
    bool hasclosed_; 

};
#endif