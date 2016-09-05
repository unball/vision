#include <vision/raw_image.hpp>

RawImage& RawImage::getInstance()
{
    static RawImage rawImage;
    return rawImage;
}

void RawImage::setRawRGBImage(const cv::Mat &raw_image)
{
    raw_image.copyTo(raw_rgb_image_);
}

cv::Mat RawImage::getRawRGBImage()
{
    return raw_rgb_image_.clone();
}

void RawImage::setRawDepthImage(const cv::Mat &raw_image)
{
    raw_image.copyTo(raw_depth_image_);
}

cv::Mat RawImage::getRawDepthImage()
{
    return raw_depth_image_.clone();
}
