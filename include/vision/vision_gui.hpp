#ifndef VISION_VISION_GUI_H_
#define VISION_VISION_GUI_H_

#include <opencv2/opencv.hpp>

#include <ros/ros.h>

class VisionGUI
{
  public:
    static VisionGUI& getInstance();

    void setInitialRGBImage(const cv::Mat &raw_image);
    void setInitialDepthImage(const cv::Mat &raw_image);

    cv::Mat getOutputRGBImage();
    cv::Mat getOutputDepthImage();

    void showOutputImages();

  private:
    VisionGUI();

    cv::Mat output_rgb_image_;
    cv::Mat output_depth_image_;

    bool using_rgb_, using_depth_;
};

#endif // VISION_VISION_GUI_H_
