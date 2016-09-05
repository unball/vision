/**
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief  SegmentationAlgorithm class
 */

#include <vision/segmentation_algorithm.hpp>

cv::Mat SegmentationAlgorithm::getSegmentationRGBOutput()
{
    return output_rgb_image_;
}

cv::Mat SegmentationAlgorithm::getSegmentationDepthOutput()
{
    return output_depth_image_;
}

std::string SegmentationAlgorithm::getFullName()
{
    return name_ + arguments_;
}

bool SegmentationAlgorithm::isName(std::string name)
{
    return name == getFullName();
}
