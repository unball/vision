#include <vision/identification_algorithm.hpp>

IdentificationAlgorithm::IdentificationAlgorithm()
{
    name_ = "IdentificationAlgorithm_BaseClass";
}

bool IdentificationAlgorithm::isName(std::string name)
{
    return name == name_;
}
