#include <vision/raw_image.hpp>

RawImage& RawImage::getInstance()
{
    static RawImage rawImage;
    return rawImage;
}

void RawImage::setRawImage(const cv::Mat &raw_image)
{
    raw_image.copyTo(raw_image_);
}

cv::Mat RawImage::getRawImage()
{
    return raw_image_;
}
