#include <vision/vision_gui.hpp>

const std::string OUTPUT_RGB_IMAGE_NAME = "output rgb image";
const std::string OUTPUT_DEPTH_IMAGE_NAME = "output depth image";

VisionGUI& VisionGUI::getInstance()
{
    static VisionGUI visionGUI;
    return visionGUI;
}

VisionGUI::VisionGUI()
{
    cv::namedWindow(OUTPUT_RGB_IMAGE_NAME);
    cv::namedWindow(OUTPUT_DEPTH_IMAGE_NAME);
    ros::param::get("/image/using_rgb", using_rgb_);
    ros::param::get("/image/using_depth", using_depth_);
}

void VisionGUI::setInitialRGBImage(const cv::Mat &raw_image)
{
    raw_image.copyTo(output_rgb_image_);
}

cv::Mat VisionGUI::getOutputRGBImage()
{
    return output_rgb_image_;
}

void VisionGUI::setInitialDepthImage(const cv::Mat &raw_image)
{
    raw_image.copyTo(output_depth_image_);
}

cv::Mat VisionGUI::getOutputDepthImage()
{
    return output_depth_image_;
}

void VisionGUI::showOutputImages()
{
    if (using_rgb_)
        cv::imshow(OUTPUT_RGB_IMAGE_NAME, output_rgb_image_);
    if (using_depth_)
        cv::imshow(OUTPUT_DEPTH_IMAGE_NAME, output_depth_image_);
    cv::waitKey(1);
}
