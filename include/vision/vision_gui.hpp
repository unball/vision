#ifndef VISION_VISION_GUI_H_
#define VISION_VISION_GUI_H_

#include <opencv2/opencv.hpp>

#include <ros/ros.h>

class VisionGUI
{
  public:
    static VisionGUI& getInstance();

    void setInitialRGBImage(const cv::Mat &raw_image);

    cv::Mat getOutputRGBImage();

    void showOutputImages();

  private:
    VisionGUI();

    cv::Mat output_rgb_image_;

    bool using_rgb_;
};

#endif // VISION_VISION_GUI_H_
