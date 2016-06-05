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

#include <ros/ros.h>

#include <vision/segmentation_algorithm.hpp>
#include <vision/identification_algorithm.hpp>

template<typename T>
std::shared_ptr<SegmentationAlgorithm> newSegmentationAlgorithm() { return std::make_shared<T>(); }

template<typename T>
std::shared_ptr<IdentificationAlgorithm> newIdentificationAlgorithm() { return std::make_shared<T>(); }

class AlgorithmFactory
{
  public:
    static std::shared_ptr<SegmentationAlgorithm> makeSegmentationAlgorithm(std::string algorithm_name);
    static std::shared_ptr<IdentificationAlgorithm> makeIdentificationAlgorithm(std::string algorithm_name);

  protected:
    static std::unordered_map<std::string, std::shared_ptr<SegmentationAlgorithm>(*)()> segmentation_map;
    static std::unordered_map<std::string, std::shared_ptr<IdentificationAlgorithm>(*)()> identification_map;
};

template<typename T>
class SegmentationRegister : AlgorithmFactory
{
  public:
    SegmentationRegister(const std::string& name)
    {
        segmentation_map[name] = &newSegmentationAlgorithm<T>;
    }
};

template<typename T>
class IdentificationRegister : AlgorithmFactory
{
  public:
    IdentificationRegister(const std::string& name)
    {
        identification_map[name] = &newIdentificationAlgorithm<T>;
    }
};

#endif // VISION_ALGORITHM_FACTORY_H_
