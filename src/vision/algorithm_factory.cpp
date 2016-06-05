#include <vision/algorithm_factory.hpp>

AlgFactoryMapType::seg_map AlgorithmFactory::segmentation_map;
AlgFactoryMapType::id_map AlgorithmFactory::identification_map;

std::shared_ptr<SegmentationAlgorithm> AlgorithmFactory::makeSegmentationAlgorithm(std::string algorithm_name)
{
    if (segmentation_map.find(algorithm_name) == segmentation_map.end())
    {
        ROS_ERROR("[AlgorithmFactory]makeSegmentationAlgorithm: Algorithm does not exist: %s",
            algorithm_name.c_str());
        exit(1);
    }
    return segmentation_map[algorithm_name]();
}

std::shared_ptr<IdentificationAlgorithm> AlgorithmFactory::makeIdentificationAlgorithm(std::string algorithm_name)
{
    if (identification_map.find(algorithm_name) == identification_map.end())
    {
        ROS_ERROR("[AlgorithmFactory]makeIdentificationAlgorithm: Algorithm does not exist: %s",
            algorithm_name.c_str());
        exit(1);
    }
    return identification_map[algorithm_name]();
}
