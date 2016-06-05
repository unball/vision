#include <vision/identifier.hpp>

Identifier& Identifier::getInstance()
{
    static Identifier identifier;
    return identifier;
}

void Identifier::runIdentificationAlgorithms()
{
    ROS_ERROR("[Identifier]: Executing %d algorithms", (int)algorithms_.size());
    for(auto algorithm : algorithms_)
        algorithm -> run();
}

void Identifier::addIdentificationAlgorithm(std::shared_ptr<IdentificationAlgorithm> algorithm)
{
    algorithms_.push_back(algorithm);
}

std::shared_ptr<IdentificationAlgorithm> Identifier::searchIdentificationAlgorithm(std::string name)
{
    for (auto algorithm : algorithms_)
        if (algorithm->isName(name))
            return algorithm;
    return NULL;
}
