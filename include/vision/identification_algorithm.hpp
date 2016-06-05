#ifndef VISION_IDENTIFICATION_ALGORITHM_H_
#define VISION_IDENTIFICATION_ALGORITHM_H_

#include <string>
#include <memory>

#include <vision/identification_output.hpp>
#include <vision/segmentation_algorithm.hpp>

class IdentificationAlgorithm
{
  public:
    IdentificationAlgorithm();
    virtual ~IdentificationAlgorithm() {}

    virtual void init() {}

    virtual void run() = 0;

    bool isName(std::string name);

    void setSegmentationAlgorithm(std::shared_ptr<SegmentationAlgorithm> seg_alg);

    void setArguments(std::string arguments);

  protected:
    std::shared_ptr<SegmentationAlgorithm> segmentation_algorithm_;

    std::string name_;
    std::string arguments_;
    IdentificationOutput output_info_;
};

#endif // VISION_IDENTIFICATION_ALGORITHM_H_
