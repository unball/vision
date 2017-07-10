#include <vision/dummy_segmentation_algorithm.hpp>

REGISTER_ALGORITHM_DEF(DummySegmentationAlgorithm);

void DummySegmentationAlgorithm::run()
{
    output_image_ = RawImage::getInstance().getRawImage();
}
