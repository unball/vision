#include <vision/dummy_identification_algorithm.hpp>

REGISTER_ALGORITHM_DEF(DummyIdentificationAlgorithm);

void DummyIdentificationAlgorithm::run()
{
    output_info_ = std::make_shared<IdentificationOutput>();
}
