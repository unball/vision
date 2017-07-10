#include <vision/vision_gui.hpp>

const std::string OUTPUT_IMAGE_NAME = "output image";

VisionGUI& VisionGUI::getInstance()
{
    static VisionGUI visionGUI;
    return visionGUI;
}

VisionGUI::VisionGUI()
{
    cv::namedWindow(OUTPUT_IMAGE_NAME);
    ros::param::get("/image/using_rgb", using_rgb_);
}

void VisionGUI::setInitialImage(const cv::Mat &raw_image)
{
    raw_image.copyTo(output_image_);
}

cv::Mat VisionGUI::getOutputImage()
{
    return output_image_;
}

void VisionGUI::showOutputImages()
{
    if (using_rgb_)
        cv::imshow(OUTPUT_IMAGE_NAME, output_image_);
    cv::waitKey(1);
}
