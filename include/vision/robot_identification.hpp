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
    void identify(std::vector<cv::Point> contour, int index);
    void findOrientation(cv::Mat mask);
    
    std::vector<cv::Rect> robots_;
    std::vector<cv::Point2f> robots_coord_;
    cv::Mat mask_;

    std::vector<cv::Vec4i> hierarchy_;

    std::string window_name_;

    cv::string allies_;
    cv::string enemies_;

    int area_ = 1155;
    bool hasclosed_; 

};
#endif