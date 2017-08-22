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
#include <cstdio>

TrackedObject::TrackedObject(std::string name)
{
    name_ = name;
    counter_ = 0;
    weight_ = 0.3;
    last_pose_vector_ = std::vector<cv::Point2f>();
    last_orientation_vector_ = std::vector<float>();
    cv::Scalar opponent_color_ = cv::Scalar(255, 255, 255);

}

std::vector<cv::Point2f> TrackedObject::getPositionVector(){
    return position_;
}

std::vector<float> TrackedObject::getOrientationVector(){
    return orientation_;    
}

void TrackedObject::runTracking()   
{


    auto id_output = identification_algorithm_->getIdentificationOutput();
    auto pose_vector = id_output->object_pose;
    auto orientation_vector = id_output->object_orientation;
    cv::Mat rgb_output = VisionGUI::getInstance().getOutputRGBImage();
    
    std::vector<float> module(last_pose_vector_.size());
    std::vector<float> module_aux(last_pose_vector_.size());
    std::vector<trackParams> params(last_pose_vector_.size());
    
    if (name_ == "ball")
    {
        position_ = pose_vector;
        orientation_ = std::vector<float>();
        cv::circle(rgb_output, pose_vector[0], 10, cv::Scalar(255,0,0));
    }
    else{
        for (int j = 0; j < pose_vector.size(); j++)
        {
            // for (int i = 0; i < pose_vector.size(); i++)
            // {
            //     if (pose_vector[i].x != -1 && pose_vector[i].y != -1)
            //     {
            //         auto vector_ = cv::Point2f(last_pose_vector_[j].x - pose_vector[i].x, last_pose_vector_[j].y - pose_vector[i].y);
            //         module[i] = sqrt(pow(vector_.x, 2) + pow(vector_.y, 2));
            //         module_aux[i] = module[i];
            //     }
            // }
            // auto minDist = *std::min_element(module_aux.begin(), module_aux.end());
            // for (int i = 0; i < module.size(); i++){
            //     if(module[i] == minDist){
            //         last_pose_vector_[i] = pose_vector[i];
            //         last_orientation_vector_[i] = orientation_vector[i];
            //         module_aux[i] = 1000000;
            //         pose_vector[i] = cv::Point2f(-1, -1);
            //     }
            // }
            // last_pose_vector_[j] = pose_vector[j];
            // last_orientation_vector_[j] = orientation_vector[j];

            if (name_ == "our_robots")
            {   
                cv::Point point1 = cv::Point(pose_vector[j].x-10, pose_vector[j].y-10);
                cv::Point point2 = cv::Point(pose_vector[j].x+10, pose_vector[j].y+10);
                switch (j){
                    case 0:
                        cv::rectangle(rgb_output, point1, point2, cv::Scalar(255, 0, 0), 3, 8, 0); 
                        break;
                    case 1:
                        cv::rectangle(rgb_output, point1, point2, cv::Scalar(0, 255, 0), 3, 8, 0); 
                        break;
                    case 2:
                        cv::rectangle(rgb_output, point1, point2, cv::Scalar(0, 0, 255), 3, 8, 0); 
                        break;
                    default:
                        break;
                }
                if (orientation_vector[j] != orientation_vector[j])
                {   
                    orientation_vector[j] = last_orientation_vector_[j];
                }
                
                // ROS_INFO("Orientation of %d: %f", j, orientation_vector[j]);
                // ROS_INFO("\n");
                cv::Point2f orient = cv::Point2f(pose_vector[j].x + 10*cos(orientation_vector[j]), pose_vector[j].y - 10*sin(orientation_vector[j]));
                // ROS_INFO("center = %f, %f", pose_vector[j].x, pose_vector[j].y);
                // ROS_INFO("p2 = %f %f", orient.x, orient.y);
                // ROS_INFO("\n");
                cv::line(rgb_output, pose_vector[j], orient, cv::Scalar(133,255,20));
            }
            else if (name_ == "opponent_robots")
            {   
                orientation_vector[j] = 0;
                cv::Point point1 = cv::Point(pose_vector[j].x-10, pose_vector[j].y-10);
                cv::Point point2 = cv::Point(pose_vector[j].x+10, pose_vector[j].y+10);
                cv::rectangle(rgb_output, point1, point2, cv::Scalar(133, 0, 133), 3, 8, 0); 
            }
        }
        position_ = pose_vector;
        
        orientation_ = orientation_vector;
    }
    last_pose_vector_ = pose_vector;
    last_orientation_vector_ = orientation_vector;
}

bool TrackedObject::isName(std::string name)
{
    return name == name_;
}

std::string TrackedObject::getName()
{
    return name_;
}

void TrackedObject::setIdentificationAlgorithm(std::shared_ptr<IdentificationAlgorithm> id_alg)
{
    identification_algorithm_ = id_alg;
}


void TrackedObject::predict(trackParams *param)
{
    param->previous_pose_ = param->predicted_pose_;

    param->predicted_pose_ += param->predicted_velocity_*1.25;
}

void TrackedObject::update(trackParams *param, cv::Point2f measured_pose)
{
    param->previous_pose_ = param->predicted_pose_;
    cv::Point2f pose_discrepancy;

    param->predicted_pose_ = weight_ * param->predicted_pose_ +  (1 - weight_) * measured_pose;
    param->predicted_velocity_ = (param->predicted_pose_ - param->previous_pose_);

}  

void TrackedObject::resetFilter(trackParams *param)
{
    param->predicted_velocity_ = cv::Point2f(0,0);
    param->predicted_pose_ = cv::Point2f(320, 240);
}

void TrackedObject::resetLastPose(trackParams *param)
{
    param->predicted_velocity_ = cv::Point2f(0,0);
    param->predicted_pose_ = cv::Point2f(param->previous_pose_);
}

bool TrackedObject::isOutOfLimits(cv::Point2f position)
{
    if(position.x < 0 or position.y < 0 or
       position.x > 640 or position.y > 480)
        return true;
    else 
        return false;
}
