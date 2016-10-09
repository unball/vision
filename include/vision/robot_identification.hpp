#ifndef IDENTIFICATION_H
#define IDENTIFICATION_H

#include <ros/ros.h>
#include <ros/package.h>
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
    cv::FileStorage colorReader_;
    cv::Mat blue_mat_;
    cv::Mat yellow_mat_;

    cv::Mat blueMask_;
    cv::Mat yellowMask_;

    cv::Mat rgb_input_;
    cv::Mat depth_input_;
    std::vector<std::vector<cv::Point>> contours_;
    std::vector<cv::Vec4i> hierarchy_;
    cv::string allies_;
    cv::string enemies_;

    int area_ = 1155;
    bool hasclosed_; 

};
#endif