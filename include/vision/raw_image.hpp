#ifndef VISION_RAW_IMAGE_H_
#define VISION_RAW_IMAGE_H_

#include <opencv2/opencv.hpp>

class RawImage
{
  public:
    static RawImage& getInstance();

    void setRawRGBImage(const cv::Mat &raw_image);
    cv::Mat getRawRGBImage();

    void setRawDepthImage(const cv::Mat &raw_image);
    cv::Mat getRawDepthImage();

  private:
    cv::Mat raw_rgb_image_;
    cv::Mat raw_depth_image_;
};

#endif // VISION_RAW_IMAGE_H_
