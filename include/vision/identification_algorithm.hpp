#ifndef UNBALL_IDENTIFICATION_ALGORITHM_H_
#define UNBALL_IDENTIFICATION_ALGORITHM_H_

#include <string>
#include <memory>

#include <vision/identification_output.hpp>
#include <vision/segmentation_algorithm.hpp>

class IdentificationAlgorithm
{
  public:
    IdentificationAlgorithm();
    virtual ~IdentificationAlgorithm() {}

    virtual void run() = 0;

    bool isName(std::string name);

  protected:
    std::shared_ptr<SegmentationAlgorithm> segmentation_algorithm_;

    std::string name_;
    IdentificationOutput output_info_;
};

#endif // UNBALL_IDENTIFICATION_ALGORITHM_H_
