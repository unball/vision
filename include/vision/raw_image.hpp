#ifndef VISION_RAW_IMAGE_H_
#define VISION_RAW_IMAGE_H_

#include <opencv2/opencv.hpp>

class RawImage
{
  public:
    static RawImage& getInstance();

    void setRawImage(const cv::Mat &raw_image);
    cv::Mat getRawImage();

  private:
    cv::Mat raw_image_;
};

#endif // VISION_RAW_IMAGE_H_
