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

    void setSegmentationAlgorithms(std::vector<std::shared_ptr<SegmentationAlgorithm>> seg_alg);

    std::shared_ptr<IdentificationOutput> getIdentificationOutput();

  protected:
    std::vector<std::shared_ptr<SegmentationAlgorithm>> segmentation_algorithms_;

    std::shared_ptr<IdentificationOutput> output_info_;
};

#endif // VISION_IDENTIFICATION_ALGORITHM_H_
