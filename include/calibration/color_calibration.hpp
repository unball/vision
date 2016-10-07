#ifndef COLOR_CALIBRATION_
#define COLOR_CALIBRATION_

#include <ros/ros.h>
#include <ros/package.h>
#include <opencv2/opencv.hpp>
#include <string>

class ColorCalibration
{
  public:
    ColorCalibration();
    ~ColorCalibration();
    void calibrate(cv::Mat rgb_input);
    bool isCalibrated();
 private:
    void saveYellow();
    void saveBlue();

    cv::FileStorage old_blue, old_yellow, old_orange;

    int hsv_min_h_, hsv_max_h_;
    int hsv_min_s_, hsv_max_s_;
    int hsv_min_v_, hsv_max_v_;
    bool is_blue_saved_, is_yellow_saved_, is_orange_saved_;
    bool calibrate_;
    std::string window_name_;
    cv::Mat segmented_image_;
    cv::FileStorage colorManager_;
};


#endif