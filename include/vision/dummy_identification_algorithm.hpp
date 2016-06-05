#ifndef VISION_DUMMY_IDENTIFICATION_ALGORITHM_H_
#define VISION_DUMMY_IDENTIFICATION_ALGORITHM_H_

#include <vision/identification_algorithm.hpp>
#include <vision/vision_gui.hpp>
#include <vision/algorithm_factory.hpp>

class DummyIdentificationAlgorithm : public IdentificationAlgorithm
{
  public:
    DummyIdentificationAlgorithm();

    void run();

  private:
    static AlgorithmRegister<DummyIdentificationAlgorithm> reg;
};

#endif // VISION_DUMMY_IDENTIFICATION_ALGORITHM_H_
