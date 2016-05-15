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
