#ifndef FIND_ROBOTS_H
#define FIND_ROBOTS_H

#include <ros/ros.h>

#include <opencv2/opencv.hpp>
#include <vision/algorithm_factory.hpp>

class FindRobots : public SegmentationAlgorithm
{
public:
    
    ALGORITHM_TYPE(FindRobots);
    void run();
    void init();
    std::vector<cv::Rect> getRobots();
    
private:
    REGISTER_ALGORITHM_DEC(FindRobots);
    cv::Mat preProcessor(cv::Mat);
    void extractPoints(cv::Mat&);
    
    cv::Mat input_;
    std::vector<cv::Rect> robots;
    int area_;

    int _cannythresh1;
    int _cannythresh2;
    int erosion_size = 1;
    int erosion_size2 = 1;
    bool hasclosed_;
    
    
};
#endif