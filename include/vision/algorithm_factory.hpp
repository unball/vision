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
#include <functional>

#include <ros/ros.h>

#include <vision/segmentation_algorithm.hpp>
#include <vision/identification_algorithm.hpp>

namespace AlgFactoryMapType
{
    typedef std::unordered_map<std::string, std::function<std::shared_ptr<SegmentationAlgorithm>()>> seg_map;
    typedef std::unordered_map<std::string, std::function<std::shared_ptr<IdentificationAlgorithm>()>> id_map;
}

class AlgorithmFactory
{
  public:
    static std::shared_ptr<SegmentationAlgorithm> makeSegmentationAlgorithm(std::string algorithm_name);
    static std::shared_ptr<IdentificationAlgorithm> makeIdentificationAlgorithm(std::string algorithm_name);

  protected:
    static AlgFactoryMapType::seg_map segmentation_map;
    static AlgFactoryMapType::id_map identification_map;
};

template<typename T>
class SegmentationRegister : AlgorithmFactory
{
  public:
    SegmentationRegister(const std::string& name)
    {
        segmentation_map[name] = std::make_shared<T>;
    }
};

template<typename T>
class IdentificationRegister : AlgorithmFactory
{
  public:
    IdentificationRegister(const std::string& name)
    {
        identification_map[name] = std::make_shared<T>;
    }
};

#endif // VISION_ALGORITHM_FACTORY_H_
