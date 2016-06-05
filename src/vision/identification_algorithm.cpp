#include <vision/identification_algorithm.hpp>

IdentificationAlgorithm::IdentificationAlgorithm()
{
    name_ = "IdentificationAlgorithm_BaseClass";
}

bool IdentificationAlgorithm::isName(std::string name)
{
    return name == name_ + arguments_ + segmentation_algorithm_->getFullName();
}

void IdentificationAlgorithm::setSegmentationAlgorithm(std::shared_ptr<SegmentationAlgorithm> seg_alg)
{
    segmentation_algorithm_ = seg_alg;
}

void IdentificationAlgorithm::setArguments(std::string arguments)
{
    arguments_ = arguments;
}
