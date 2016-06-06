#include <vision/dummy_identification_algorithm.hpp>

REGISTER_ALGORITHM_DEF(DummyIdentificationAlgorithm);

void DummyIdentificationAlgorithm::run()
{
    output_info_.object_center = cv::Point(10,10);
    cv::Mat output_img = VisionGUI::getInstance().getOutputImage();
    cv::circle(output_img, output_info_.object_center, 5, cv::Scalar(255, 0, 0));
}
