#ifndef DEPTH_FIX_H
#define DEPTH_FIX_H

#include <opencv2/opencv.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <ros/ros.h>

class DepthFix
{
  public:
    DepthFix();
    cv::Mat fix(cv::Mat &depth_frame);    
  private:
    void adjustNoise(cv::Mat &image);
    int noise_thresh_;
    const int max_BINARY_value_ = 255;
};



#endif