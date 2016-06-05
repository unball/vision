#ifndef VISION_DUMMY_SEGMENTATION_ALGORITHM_H_
#define VISION_DUMMY_SEGMENTATION_ALGORITHM_H_

#include <ros/ros.h>

#include <vision/raw_image.hpp>
#include <vision/segmentation_algorithm.hpp>

class DummySegmentationAlgorithm : public SegmentationAlgorithm
{
  public:
    DummySegmentationAlgorithm();

    void run();
};

#endif // VISION_DUMMY_SEGMENTATION_ALGORITHM_H_
