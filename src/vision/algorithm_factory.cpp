#include <vision/algorithm_factory.hpp>

AlgorithmFactory::alg_map AlgorithmFactory::algorithm_map;

std::shared_ptr<SegmentationAlgorithm> AlgorithmFactory::makeSegmentationAlgorithm(std::string algorithm_name)
{
    if (algorithm_map.find(algorithm_name) == algorithm_map.end())
        factoryError("makeSegmentationAlgorithm", algorithm_name);

    // Instantiate and dynamically downcast a segmentation algorithm
    std::shared_ptr<SegmentationAlgorithm> seg_alg;
    seg_alg = std::dynamic_pointer_cast<SegmentationAlgorithm>(algorithm_map[algorithm_name]());

    if (not seg_alg)
        factoryError("makeSegmentationAlgorithm", algorithm_name);

    return seg_alg;
}

std::shared_ptr<IdentificationAlgorithm> AlgorithmFactory::makeIdentificationAlgorithm(std::string algorithm_name)
{
    if (algorithm_map.find(algorithm_name) == algorithm_map.end())
        factoryError("makeIdentificationAlgorithm", algorithm_name);

    // Instantiate and dynamically downcast an identification algorithm
    std::shared_ptr<IdentificationAlgorithm> id_alg;
    id_alg = std::dynamic_pointer_cast<IdentificationAlgorithm>(algorithm_map[algorithm_name]());

    if (not id_alg)
        factoryError("makeIdentificationAlgorithm", algorithm_name);

    return id_alg;
}

void AlgorithmFactory::factoryError(std::string method, std::string algorithm_name)
{
    ROS_ERROR("[AlgorithmFactory]%s: Algorithm does not exist: %s", method.c_str(), algorithm_name.c_str());
    exit(1);
}
