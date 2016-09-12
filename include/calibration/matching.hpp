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
    bool isDone();
    void showFrames(cv::Mat rbg_frame, cv::Mat depth_frame);
  
  private:
    void close();
    bool is_done_;
    cv::Mat matching_matrix_;
    CapturePoints depth_cap_, rbg_cap_;
    std::vector<cv::Point2f> rgb_points_, depth_points_;
    std::string rgb_window_, depth_window_;
};


#endif