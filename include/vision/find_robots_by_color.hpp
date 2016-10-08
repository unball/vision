#ifndef FIND_ROBOTS_BY_COLOR_
#define FIND_ROBOTS_BY_COLOR_H

#include <ros/ros.h>
#include <ros/package.h>
#include <opencv2/opencv.hpp>
#include <vision/algorithm_factory.hpp>
#include <vision/raw_image.hpp>

class FindRobotsColor : public SegmentationAlgorithm
{
public:
    
    ALGORITHM_TYPE(FindRobotsColor);
    void run();
    void init();
    
private:
    REGISTER_ALGORITHM_DEC(FindRobotsColor);
    void preProcessor(cv::Mat);

    std::string window_name_ = "Find Robots";
    std::vector<std::vector<cv::Point>> contours_;
    std::vector<cv::Vec4i> hierarchy_;


    cv::FileStorage color_reader_;
    cv::Mat blue_mat_, yellow_mat_;
    
    cv::Mat rgb_input_;
    cv::Mat yellow_mask_;
    cv::Mat blue_mask_;
    
    int erosion_size = 1;
    int erosion_size2 = 1;    
    bool hasclosed_;
};
#endif