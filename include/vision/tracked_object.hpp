/**
 * @file   tracked_object.hpp
 * @author Gabriel Naves da Silva
 * @date   22/03/2016
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief  TrackedObject class
 *
 * The TrackedObject class defines the attributes and functionality
 * common to all objects that require tracking, like the robots and the ball.
 */

#ifndef VISION_TRACKED_OBJECT_H_
#define VISION_TRACKED_OBJECT_H_

#include <string>
#include <memory>
#include <cmath>
#include <cstdio>

#include <opencv2/opencv.hpp>
#include <vision/identification_algorithm.hpp>
#include <vision/vision_gui.hpp>

class TrackedObject
{
  public:
    TrackedObject(std::string name = "DefaultTrackedObject");

    virtual void runTracking();

    bool isName(std::string name);
    std::string getName();

    void setIdentificationAlgorithm(std::shared_ptr<IdentificationAlgorithm> id_alg);

    std::vector<cv::Point2f> getPositionVector();
    std::vector<float> getOrientationVector();
    std::vector<bool> getFoundObjectsVector();

  private:
    void drawTrackedObject();

  protected:
    std::string name_;
    std::shared_ptr<IdentificationAlgorithm> identification_algorithm_;
    std::vector<cv::Point2f> position_;
    std::vector<float> orientation_;
    std::vector<bool> found_objects_;
};

#endif // VISION_TRACKED_OBJECT_H_
