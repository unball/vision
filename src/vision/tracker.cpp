/**
 * @file   tracker.cpp
 * @author Gabriel Naves da Silva
 * @date   22/03/2016
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Tracker class
 */

#include <vision/tracker.hpp>

Tracker& Tracker::getInstance()
{
    static Tracker tracker;
    return tracker;
}

void Tracker::runTracking()
{
    ROS_DEBUG("Tracking %d objects", (int)tracked_objects_.size());
    for(auto object : tracked_objects_)
        object->runTracking();
}

void Tracker::addTrackedObject(std::shared_ptr<TrackedObject> obj)
{
    tracked_objects_.push_back(obj);
}

std::unordered_map<std::string, TrackingOutput> Tracker::getTrackerOutput()
{
    std::unordered_map<std::string, TrackingOutput> result;
    for (auto object : tracked_objects_)
    {
        TrackingOutput tracking_output;
        tracking_output.positions = object->getPositionVector();
        result[object->getName()] = tracking_output;
    }
    return result;
}
