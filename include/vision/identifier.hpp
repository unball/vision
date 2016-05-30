#ifndef UNBALL_IDENTIFIER_H_
#define UNBALL_IDENTIFIER_H_

#include <vector>
#include <memory>

#include <vision/identification_algorithm.hpp>

class Identifier
{
  public:
    static Identifier& getInstance();

    void runIdentificationAlgorithms();

    void addIdentificationAlgorithm(std::shared_ptr<IdentificationAlgorithm> algorithm);

  private:
    std::vector<std::shared_ptr<IdentificationAlgorithm>> algorithms_;
};

#endif // UNBALL_IDENTIFIER_H_
