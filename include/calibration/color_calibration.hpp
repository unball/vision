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
    void calibrate(cv::Mat& rgb_input);
    cv::Mat getRGBCalibrated();
    bool isCalibrated();
 private:
    void save(std::string);
    void equalizeIntensity(cv::Mat& rgb_input);
    cv::Mat old_blue, old_yellow, old_orange, old_robot0, old_robot1, old_robot2;
    cv::Mat rgb_calibrated_;

    int hsv_min_h_, hsv_max_h_;
    int hsv_min_s_, hsv_max_s_;
    int hsv_min_v_, hsv_max_v_;
    int hdr_;
    bool is_blue_saved_, is_yellow_saved_, is_orange_saved_, is_robot0_saved_, is_robot1_saved_, is_robot2_saved_;
    bool calibrate_;

    std::string window_name_;
    std::string window_name_HDR_;
    cv::Mat segmented_image_;
    cv::FileStorage colorManager_;
    cv::FileStorage colorHandler_;
};


#endif