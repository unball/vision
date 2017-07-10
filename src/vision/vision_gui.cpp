#include <vision/vision_gui.hpp>

const std::string OUTPUT_RGB_IMAGE_NAME = "output rgb image";

VisionGUI& VisionGUI::getInstance()
{
    static VisionGUI visionGUI;
    return visionGUI;
}

VisionGUI::VisionGUI()
{
    cv::namedWindow(OUTPUT_RGB_IMAGE_NAME);
    ros::param::get("/image/using_rgb", using_rgb_);
}

void VisionGUI::setInitialRGBImage(const cv::Mat &raw_image)
{
    raw_image.copyTo(output_rgb_image_);
}

cv::Mat VisionGUI::getOutputRGBImage()
{
    return output_rgb_image_;
}

void VisionGUI::showOutputImages()
{
    if (using_rgb_)
        cv::imshow(OUTPUT_RGB_IMAGE_NAME, output_rgb_image_);
    cv::waitKey(1);
}
