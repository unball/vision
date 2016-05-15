#ifndef UNBALL_DUMMY_IDENTIFICATION_ALGORITHM_H_
#define UNBALL_DUMMY_IDENTIFICATION_ALGORITHM_H_

#include <vision/identification_algorithm.hpp>

class DummyIdentificationAlgorithm : public IdentificationAlgorithm
{
  public:
    DummyIdentificationAlgorithm();

    void run();
};

#endif // UNBALL_DUMMY_IDENTIFICATION_ALGORITHM_H_
