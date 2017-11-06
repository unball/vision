#ifndef VISION_HSV_IN_RANGE_COLOR_SEGMENTATION_H_
#define VISION_HSV_IN_RANGE_COLOR_SEGMENTATION_H_

#include <ros/ros.h>
#include <ros/package.h>

#include <vision/raw_image.hpp>
#include <vision/segmentation_algorithm.hpp>
#include <vision/algorithm_factory.hpp>

class HSVInRangeColorSegmentation : public SegmentationAlgorithm
{
  public:
    ALGORITHM_TYPE(HSVInRangeColorSegmentation);

    void init();
    void run();

  private:
    REGISTER_ALGORITHM_DEC(HSVInRangeColorSegmentation);

    std::string window_name_;
    cv::FileStorage colorReader_;
    cv::Mat color_mat_;
    int kernel_size_ = 1;
};

#endif // VISION_HSV_IN_RANGE_COLOR_SEGMENTATION_H_
