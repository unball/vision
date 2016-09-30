#ifndef FIND_ROBOTS_H
#define FIND_ROBOTS_H

#include <ros/ros.h>

#include <opencv2/opencv.hpp>
#include <vision/algorithm_factory.hpp>
#include <vision/raw_image.hpp>

class FindRobots : public SegmentationAlgorithm
{
public:
    
    ALGORITHM_TYPE(FindRobots);
    void run();
    void init();
    
private:
    REGISTER_ALGORITHM_DEC(FindRobots);
    cv::Mat preProcessor(cv::Mat);
    std::string window_name_ = "Find Robots";

    cv::Mat input_;
    std::vector<cv::Rect> robots;
    int area_;

    int cannythresh1_;
    int cannythresh2_;
    
    
};
#endif