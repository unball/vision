#ifndef VISION_BALL_IN_RANGE_COLOR_SEGMENTATION_H_
#define VISION_BALL_IN_RANGE_COLOR_SEGMENTATION_H_

#include <ros/ros.h>
#include <ros/package.h>
#include <vision/raw_image.hpp>
#include <vision/segmentation_algorithm.hpp>
#include <vision/algorithm_factory.hpp>

class BallInRangeColorSegmentation : public SegmentationAlgorithm
{
  public:
    ALGORITHM_TYPE(BallInRangeColorSegmentation);

    void init();
    void run();

  private:
    REGISTER_ALGORITHM_DEC(BallInRangeColorSegmentation);
    cv::FileStorage colorReader_;
    std::string window_name_ = "Ball segmetation with inrange";
    std::vector<float> hsv_min_;
    cv::Scalar hsv_max_;
    int kernel_size_ = 5;
};

#endif // VISION_BALL_IN_RANGE_COLOR_SEGMENTATION_H_
