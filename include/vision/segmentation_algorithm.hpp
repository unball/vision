/**
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief  SegmentationAlgorithm class
 *
 * Defines the SegmentationAlgorithm class
 *
 * The SegmentationAlgorithm class defines a common interface to all different segmentation
 * algorithms that may be implemented within this project.
 */

#ifndef VISION_SEGMENTATION_ALGORITHM_H_
#define VISION_SEGMENTATION_ALGORITHM_H_

#include <string>

#include <opencv2/opencv.hpp>

#include <vision/algorithm.hpp>

class SegmentationAlgorithm : public Algorithm
{
  public:
    virtual ~SegmentationAlgorithm() {}

    cv::Mat getSegmentationRGBOutput();
    cv::Mat getSegmentationDepthOutput();
    std::string getFullName();
    bool isName(std::string name);

  protected:
    cv::Mat output_rgb_image_;
    cv::Mat output_depth_image_;
};

#endif // VISION_SEGMENTATION_ALGORITHM_H_
