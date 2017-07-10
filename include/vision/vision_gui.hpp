#ifndef VISION_VISION_GUI_H_
#define VISION_VISION_GUI_H_

#include <opencv2/opencv.hpp>

#include <ros/ros.h>

class VisionGUI
{
  public:
    static VisionGUI& getInstance();

    void setInitialImage(const cv::Mat &raw_image);

    cv::Mat getOutputImage();

    void showOutputImages();

  private:
    VisionGUI();

    cv::Mat output_image_;

    bool using_rgb_;
};

#endif // VISION_VISION_GUI_H_
