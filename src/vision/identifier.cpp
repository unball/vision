#include <vision/identifier.hpp>

Identifier& Identifier::getInstance()
{
    static Identifier identifier;
    return identifier;
}

void Identifier::runIdentificationAlgorithms()
{
    for(auto algorithm : algorithms_)
        algorithm -> run();
}

void Identifier::addIdentificationAlgorithm(std::shared_ptr<IdentificationAlgorithm> algorithm)
{
    algorithms_.push_back(algorithm);
}
