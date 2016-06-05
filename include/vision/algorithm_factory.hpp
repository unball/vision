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

class AlgorithmFactory
{
  public:
    static std::shared_ptr<SegmentationAlgorithm> makeSegmentationAlgorithm(std::string algorithm_name);
    static std::shared_ptr<IdentificationAlgorithm> makeIdentificationAlgorithm(std::string algorithm_name);

  protected:
    typedef std::unordered_map<std::string, std::function<std::shared_ptr<Algorithm>()>> alg_map;

    static alg_map algorithm_map;
};

template<typename T>
class AlgorithmRegister : AlgorithmFactory
{
  public:
    AlgorithmRegister(const std::string& name)
    {
        algorithm_map[name] = std::make_shared<T>;
    }
};

#endif // VISION_ALGORITHM_FACTORY_H_
