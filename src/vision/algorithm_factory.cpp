#include <vision/algorithm_factory.hpp>

AlgorithmFactory& AlgorithmFactory::getInstance()
{
    static AlgorithmFactory algorithm_factory;
    return algorithm_factory;
}

std::shared_ptr<SegmentationAlgorithm> AlgorithmFactory::makeSegmentationAlgorithm(std::string algorithm_name)
{
    if (segmentation_map.find(algorithm_name) == segmentation_map.end())
        return NULL;
    return segmentation_map[algorithm_name]();
}

std::shared_ptr<IdentificationAlgorithm> AlgorithmFactory::makeIdentificationAlgorithm(std::string algorithm_name)
{
    if (identification_map.find(algorithm_name) == identification_map.end())
        return NULL;
    return identification_map[algorithm_name]();
}

AlgorithmFactory::AlgorithmFactory()
{
    initSegmentationMap();
    initIdentificationMap();
}

void AlgorithmFactory::initSegmentationMap()
{
    segmentation_map["DummySegmentationAlgorithm"] = &newSegmentationAlgorithm<DummySegmentationAlgorithm>;
}

void AlgorithmFactory::initIdentificationMap()
{
    identification_map["DummyIdentificationAlgorithm"] = &newIdentificationAlgorithm<DummyIdentificationAlgorithm>;
}
