/**
 * Reference used on the making of this class:
 * http://stackoverflow.com/questions/582331/
    is-there-a-way-to-instantiate-objects-from-a-string-holding-their-class-name
 */

#ifndef VISION_ALGORITHM_FACTORY_H_
#define VISION_ALGORITHM_FACTORY_H_

#include <memory>
#include <unordered_map>
#include <string>

// Segmentation algorithms
#include <vision/dummy_segmentation_algorithm.hpp>

// Identification algorithms
#include <vision/dummy_identification_algorithm.hpp>

template<typename T>
std::shared_ptr<SegmentationAlgorithm> newSegmentationAlgorithm() { return std::make_shared<T>(); }

template<typename T>
std::shared_ptr<IdentificationAlgorithm> newIdentificationAlgorithm() { return std::make_shared<T>(); }

class AlgorithmFactory
{
  public:
    static AlgorithmFactory& getInstance();
    std::shared_ptr<SegmentationAlgorithm> makeSegmentationAlgorithm(std::string algorithm_name);
    std::shared_ptr<IdentificationAlgorithm> makeIdentificationAlgorithm(std::string algorithm_name);

  private:
    AlgorithmFactory();
    void initSegmentationMap();
    void initIdentificationMap();

    std::unordered_map<std::string, std::shared_ptr<SegmentationAlgorithm>(*)()> segmentation_map;
    std::unordered_map<std::string, std::shared_ptr<IdentificationAlgorithm>(*)()> identification_map;
};

#endif // VISION_ALGORITHM_FACTORY_H_
