#ifndef VISION_BALL_IN_RANGE_COLOR_SEGMENTATION_H_
#define VISION_BALL_IN_RANGE_COLOR_SEGMENTATION_H_

#include <ros/ros.h>

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
    std::string window_name_ = "Ball segmetation with inrange";
    int hsv_min_h_ = 170, hsv_max_h_ = 180;
    int hsv_min_s_ = 150, hsv_max_s_ = 220;
    int hsv_min_v_ = 170, hsv_max_v_ = 200;
    int kernel_size_ = 5;
};

#endif // VISION_BALL_IN_RANGE_COLOR_SEGMENTATION_H_
