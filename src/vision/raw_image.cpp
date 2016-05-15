#include <vision/raw_image.hpp>

RawImage& RawImage::getInstance()
{
    static RawImage rawImage;
    return rawImage;
}

void RawImage::setRawImage(cv::Mat raw_image)
{
    raw_image_ = raw_image;
}

cv::Mat RawImage::getRawImage()
{
    return raw_image_;
}
