#ifndef MATCHING_H_
#define MATCHING_H_

#include <opencv2/opencv.hpp>
#include <ros/ros.h>

#include <calibration/capture_points.hpp>

class Matching
{
  public:
    Matching(std::string rgb_window, std::string depth_window);
    void run();
    cv::Mat getMatrix();
  
  private:
    cv::Mat matching_matrix_;
    CapturePoints depth_cap_, rbg_cap_;
};


#endif