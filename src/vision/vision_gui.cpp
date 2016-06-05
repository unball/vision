#include <vision/vision_gui.hpp>

const std::string OUTPUT_IMAGE_NAME = "output_image_";

VisionGUI& VisionGUI::getInstance()
{
    static VisionGUI visionGUI;
    return visionGUI;
}

VisionGUI::VisionGUI()
{
    cv::namedWindow(OUTPUT_IMAGE_NAME);
}

void VisionGUI::setInitialImage(const cv::Mat &raw_image)
{
    raw_image.copyTo(output_image_);
}

cv::Mat VisionGUI::getOutputImage()
{
    return output_image_;
}

void VisionGUI::showOutputImage()
{
    cv::imshow(OUTPUT_IMAGE_NAME, output_image_);
}
