#include <vision/identification_algorithm.hpp>

bool IdentificationAlgorithm::isName(std::string name)
{
    return name == name_ + arguments_ + segmentation_algorithm_->getFullName();
}

void IdentificationAlgorithm::setSegmentationAlgorithm(std::shared_ptr<SegmentationAlgorithm> seg_alg)
{
    segmentation_algorithm_ = seg_alg;
}

std::shared_ptr<IdentificationOutput> IdentificationAlgorithm::getSegmentationOutput()
{
    return output_info_;
}
