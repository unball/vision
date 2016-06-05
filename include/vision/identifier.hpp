#ifndef VISION_IDENTIFIER_H_
#define VISION_IDENTIFIER_H_

#include <vector>
#include <memory>

#include <ros/ros.h>

#include <vision/identification_algorithm.hpp>

class Identifier
{
  public:
    static Identifier& getInstance();

    void runIdentificationAlgorithms();

    void addIdentificationAlgorithm(std::shared_ptr<IdentificationAlgorithm> algorithm);
    std::shared_ptr<IdentificationAlgorithm> searchIdentificationAlgorithm(std::string name);

  private:
    std::vector<std::shared_ptr<IdentificationAlgorithm>> algorithms_;
};

#endif // VISION_IDENTIFIER_H_
