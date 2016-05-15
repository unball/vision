#include <vision/dummy_segmentation_algorithm.hpp>

DummySegmentationAlgorithm::DummySegmentationAlgorithm()
{
    name_ = "DummySegmentationAlgorithm";
}


void DummySegmentationAlgorithm::run(cv::Mat raw_image)
{
    raw_image.copyTo(output_image_);
}
