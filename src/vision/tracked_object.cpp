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
}

std::vector<cv::Point2f> TrackedObject::getPositionVector(){
    return position_;
}

std::vector<float> TrackedObject::getOrientationVector(){
    return orientation_;
}

std::vector<bool> TrackedObject::getFoundObjectsVector()
{
    return found_objects_;
}

void TrackedObject::runTracking()
{
    std::shared_ptr<IdentificationOutput> id_output = identification_algorithm_->getIdentificationOutput();

    found_objects_ = id_output->found_object;
    if (position_.size() != id_output->object_pose.size())
        position_.resize(id_output->object_pose.size());
    if (orientation_.size() != id_output->object_orientation.size())
        orientation_.resize(id_output->object_orientation.size());

    for (int i = 0; i < position_.size(); ++i)
        if (found_objects_[i])
            position_[i] = id_output->object_pose[i];

    for (int i = 0; i < orientation_.size(); ++i)
        if (found_objects_[i])
            orientation_[i] = id_output->object_orientation[i];


    drawTrackedObject();
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

void TrackedObject::drawTrackedObject()
{
    cv::Mat image = VisionGUI::getInstance().getOutputImage();
    for (int i = 0; i < position_.size(); ++i)
        cv::circle(image, position_[i], 10, cv::Scalar(255, 0, 0), 2);

}
