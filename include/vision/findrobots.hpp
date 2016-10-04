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
    void extractPoints(cv::Mat&);
    void find(cv::Mat);

    std::string window_name_ = "Find Robots";
    std::vector<std::vector<cv::Point>> contours_;
    std::vector<cv::Vec4i> hierarchy_;

    cv::Mat input_;
    cv::Mat rgb_input_;
    cv::Mat depth_to_pub_;

    int cannythresh1_ = 40;
    int cannythresh2_ = 20;
    int erosion_size = 1;
    int erosion_size2 = 1;    
    bool hasclosed_;
};
#endif