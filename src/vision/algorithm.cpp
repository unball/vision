#include <vision/algorithm.hpp>

void Algorithm::setArguments(std::string arguments)
{
    arguments_ = arguments;
}

std::string Algorithm::getAlgorithmName()
{
    return name_;
}
