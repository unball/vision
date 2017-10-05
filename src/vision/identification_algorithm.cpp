#include <vision/identification_algorithm.hpp>

bool IdentificationAlgorithm::isName(std::string name)
{
    std::string full_name = name_ + arguments_;
    for (auto seg_alg : segmentation_algorithms_)
        full_name += seg_alg->getFullName();
    return name == full_name;
}

void IdentificationAlgorithm::setSegmentationAlgorithms(std::vector<std::shared_ptr<SegmentationAlgorithm>> seg_alg)
{
    segmentation_algorithms_ = seg_alg;
}

std::shared_ptr<IdentificationOutput> IdentificationAlgorithm::getIdentificationOutput()
{
    return output_info_;
}
