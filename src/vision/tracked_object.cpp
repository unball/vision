/**
 * @file   tracked_object.hpp
 * @author Gabriel Naves da Silva
 * @date   22/03/2016
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief  TrackedObject class
 */

#include <vision/tracked_object.hpp>

TrackedObject::TrackedObject(std::string name)
{
    name_ = name;
    counter_ = 0;
    weight_ = 0.3;
    predicted_velocity_ = cv::Point2f(0, 0);
    predicted_pose_ = cv::Point2f(320, 240);
}

std::vector<cv::Point2f> TrackedObject::getPositionVector(){
    return position_;
}

void TrackedObject::runTracking()
{
    auto id_output = identification_algorithm_->getIdentificationOutput();
    auto poseVector = id_output->object_pose;
    cv::Mat rgb_output = VisionGUI::getInstance().getOutputRGBImage();
    

    for (auto it = poseVector.begin(); it != poseVector.end(); it++)
    {
        if ((*it).x != -1 || (*it).y != -1){
            update((*it));
            object_pose_ = (*it);
        }
        else{
            predict();
            if (counter_ > 15){
                resetLastPose();
                counter_ = 0;
            }
            if((*it).x < 0 or (*it).y < 0 or
               (*it).x > 640 or (*it).y > 480)
                resetFilter();

            object_pose_ = predicted_pose_;
        }
        if(name_ == "ball"){
            cv::circle(rgb_output, object_pose_, 10, cv::Scalar(255,0,0));
        }else if (name_ == "our_robots")
        {
            cv::Point point1 = cv::Point(object_pose_.x-10, object_pose_.y-10);
            cv::Point point2 = cv::Point(object_pose_.x+10, object_pose_.y+10);
            cv::rectangle(rgb_output, point1, point2, cv::Scalar(133, 133, 133), 3, 8, 0); 
        }
        else if (name_ == "opponent_robots")
        {
            cv::Point point1 = cv::Point(object_pose_.x-10, object_pose_.y-10);
            cv::Point point2 = cv::Point(object_pose_.x+10, object_pose_.y+10);
            cv::rectangle(rgb_output, point1, point2, cv::Scalar(133, 0, 133), 3, 8, 0); 
        }
    }
}

bool TrackedObject::isName(std::string name)
{
    return name == name_;
}

void TrackedObject::setIdentificationAlgorithm(std::shared_ptr<IdentificationAlgorithm> id_alg)
{
    identification_algorithm_ = id_alg;
}


void TrackedObject::predict()
{
    predicted_pose_ += predicted_velocity_*1.25;
}

void TrackedObject::update(cv::Point2f measured_pose)
{
    previous_pose_ = predicted_pose_;
    cv::Point2f pose_discrepancy;

    predicted_pose_ = weight_ * predicted_pose_ +  (1 - weight_) * measured_pose;
    predicted_velocity_ = (predicted_pose_ - previous_pose_);
}  

void TrackedObject::resetFilter()
{
    predicted_velocity_ = cv::Point2f(0,0);
    predicted_pose_ = cv::Point2f(320, 240);
}

void TrackedObject::resetLastPose()
{
    predicted_velocity_ = cv::Point2f(0,0);
    predicted_pose_ = cv::Point2f(previous_pose_);
}
