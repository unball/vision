/**
 * @file   segmentation_algorithm.hpp
 * @author Gabriel Naves da Silva
 * @date   21/03/2016
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief  SegmentationAlgorithm class
 */

#include <vision/segmentation_algorithm.hpp>

SegmentationAlgorithm::SegmentationAlgorithm()
{
    name_ = "SegmentationAlgorithm_BaseClass";
}

cv::Mat SegmentationAlgorithm::getSegmentationOutput()
{
    return output_image_;
}

std::string SegmentationAlgorithm::getFullName()
{
    return name_ + arguments_;
}

bool SegmentationAlgorithm::isName(std::string name)
{
    return name == getFullName();
}

void SegmentationAlgorithm::setArguments(std::string arguments)
{
    arguments_ = arguments;
}
