#ifndef VISION_DUMMY_SEGMENTATION_ALGORITHM_H_
#define VISION_DUMMY_SEGMENTATION_ALGORITHM_H_

#include <ros/ros.h>

#include <vision/raw_image.hpp>
#include <vision/segmentation_algorithm.hpp>
#include <vision/algorithm_factory.hpp>

class DummySegmentationAlgorithm : public SegmentationAlgorithm
{
  public:
    ALGORITHM_TYPE(DummySegmentationAlgorithm);

    void run();

  private:
    REGISTER_ALGORITHM_DEC(DummySegmentationAlgorithm);
};

#endif // VISION_DUMMY_SEGMENTATION_ALGORITHM_H_
