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

#include <opencv2/opencv.hpp>
#include <vision/identification_algorithm.hpp>
#include <vision/vision_gui.hpp>

struct trackParams
{
    cv::Point2f predicted_velocity_ = cv::Point2f(0, 0);
    cv::Point2f predicted_pose_ = cv::Point2f(320, 240);
    cv::Point2f previous_pose_;
};

class TrackedObject
{
  public:
    TrackedObject(std::string name = "DefaultTrackedObject");

    void runTracking();

    bool isName(std::string name);
    std::string getName();

    void setIdentificationAlgorithm(std::shared_ptr<IdentificationAlgorithm> id_alg);

    std::vector<cv::Point2f> getPositionVector();
    std::vector<float> getOrientationVector();

  private:
    bool isOutOfLimits(cv::Point2f);
    
    void predict(trackParams *param);
    void update(trackParams *param, cv::Point2f measured_pose);
    void resetFilter(trackParams *param);
    void resetLastPose(trackParams *param);
    
    std::vector<cv::Point2f> last_pose_vector_;
    std::vector<float> last_orientation_vector_;

    int counter_;
    float weight_;
    cv::Scalar opponent_color_;
  protected:

    std::string name_;

    std::shared_ptr<IdentificationAlgorithm> identification_algorithm_;

    std::vector<cv::Point2f> position_;

    std::vector<float> orientation_;
};

#endif // VISION_TRACKED_OBJECT_H_
