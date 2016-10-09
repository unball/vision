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

void TrackedObject::runTracking()
{
    auto id_output = identification_algorithm_->getIdentificationOutput();

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
