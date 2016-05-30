#include <vision/identification_algorithm.hpp>

IdentificationAlgorithm::IdentificationAlgorithm()
{
    name_ = "IdentificationAlgorithm_BaseClass";
}

bool IdentificationAlgorithm::isName(std::string name)
{
    return name == name_;
}

void IdentificationAlgorithm::setSegmentationAlgorithm(std::shared_ptr<SegmentationAlgorithm> seg_alg)
{
    segmentation_algorithm_ = seg_alg;
}
