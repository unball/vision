#include <vision/algorithm_factory.hpp>

AlgorithmFactory::alg_map AlgorithmFactory::algorithm_map;

std::shared_ptr<SegmentationAlgorithm> AlgorithmFactory::makeSegmentationAlgorithm(std::string algorithm_name)
{
    if (algorithm_map.find(algorithm_name) == algorithm_map.end())
    {
        ROS_ERROR("[AlgorithmFactory]makeSegmentationAlgorithm: Algorithm does not exist: %s",
            algorithm_name.c_str());
        exit(1);
    }

    std::shared_ptr<SegmentationAlgorithm> seg_alg;
    seg_alg = std::static_pointer_cast<SegmentationAlgorithm>(algorithm_map[algorithm_name]());
    ROS_ERROR("Creating seg alg: %s", seg_alg->getAlgorithmName().c_str());
    return seg_alg;
}

std::shared_ptr<IdentificationAlgorithm> AlgorithmFactory::makeIdentificationAlgorithm(std::string algorithm_name)
{
    if (algorithm_map.find(algorithm_name) == algorithm_map.end())
    {
        ROS_ERROR("[AlgorithmFactory]makeIdentificationAlgorithm: Algorithm does not exist: %s",
            algorithm_name.c_str());
        exit(1);
    }

    std::shared_ptr<IdentificationAlgorithm> id_alg;
    id_alg = std::static_pointer_cast<IdentificationAlgorithm>(algorithm_map[algorithm_name]());
    ROS_ERROR("Creating id alg: %s", id_alg->getAlgorithmName().c_str());
    return id_alg;
}
