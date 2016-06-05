#ifndef VISION_DUMMY_IDENTIFICATION_ALGORITHM_H_
#define VISION_DUMMY_IDENTIFICATION_ALGORITHM_H_

#include <vision/identification_algorithm.hpp>
#include <vision/vision_gui.hpp>

class DummyIdentificationAlgorithm : public IdentificationAlgorithm
{
  public:
    DummyIdentificationAlgorithm();

    void run();
};

#endif // VISION_DUMMY_IDENTIFICATION_ALGORITHM_H_
