/**
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief  Algorithm class
 *
 * Defines the Algorithm class
 *
 * The Algorithm class will hold all implementation common to both segmentation and identification
 * algorithms.
 */

#ifndef VISION_ALGORITHM_H_
#define VISION_ALGORITHM_H_

#include <string>

class Algorithm
{
  public:
    virtual ~Algorithm() {}

    virtual void init() {}
    virtual void run() = 0;

    void setArguments(std::string arguments);

    std::string getAlgorithmName();

  protected:
    std::string name_;
    std::string arguments_;
};

#endif // VISION_ALGORITHM_H_
