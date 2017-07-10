#include <vision/dummy_segmentation_algorithm.hpp>

REGISTER_ALGORITHM_DEF(DummySegmentationAlgorithm);

void DummySegmentationAlgorithm::run()
{
    RawImage::getInstance().getRawRGBImage().copyTo(output_rgb_image_);
}
