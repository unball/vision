#include <vision/dummy_segmentation_algorithm.hpp>

DummySegmentationAlgorithm::DummySegmentationAlgorithm()
{
    name_ = "DummySegmentationAlgorithm";
}

void DummySegmentationAlgorithm::run()
{
    RawImage::getInstance().getRawImage().copyTo(output_image_);
}
