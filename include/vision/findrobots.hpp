#ifndef FIND_ROBOTS_H
#define FIND_ROBOTS_H

#include "opencv2/opencv.hpp"

class FindRobots
{
public:
    FindRobots();
    void find(cv::Mat);
    std::vector<cv::Rect> getRobots();

private:
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