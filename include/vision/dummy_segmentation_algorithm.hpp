#ifndef UNBALL_DUMMY_SEGMENTATION_ALGORITHM_H_
#define UNBALL_DUMMY_SEGMENTATION_ALGORITHM_H_

#include <vision/segmentation_algorithm.hpp>

class DummySegmentationAlgorithm : public SegmentationAlgorithm
{
  public:
    DummySegmentationAlgorithm();

    void run(cv::Mat raw_image);
};

#endif // UNBALL_DUMMY_SEGMENTATION_ALGORITHM_H_
