#ifndef VISION_IDENTIFICATION_ALGORITHM_H_
#define VISION_IDENTIFICATION_ALGORITHM_H_

#include <string>
#include <memory>

#include <vision/identification_output.hpp>
#include <vision/segmentation_algorithm.hpp>
#include <vision/algorithm.hpp>

class IdentificationAlgorithm : public Algorithm
{
  public:
    virtual ~IdentificationAlgorithm() {}

    bool isName(std::string name);

    void setSegmentationAlgorithm(std::shared_ptr<SegmentationAlgorithm> seg_alg);

  protected:
    std::shared_ptr<SegmentationAlgorithm> segmentation_algorithm_;

    IdentificationOutput output_info_;
};

#endif // VISION_IDENTIFICATION_ALGORITHM_H_
