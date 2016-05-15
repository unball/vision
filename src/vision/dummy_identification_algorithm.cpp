#include <vision/dummy_identification_algorithm.hpp>

DummyIdentificationAlgorithm::DummyIdentificationAlgorithm()
{
    name_ = "DummyIdentificationAlgorithm";
}

void DummyIdentificationAlgorithm::run()
{
    output_info_.object_center = cv::Point(10,10);
}
