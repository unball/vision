#include "calibration/select_field.hpp"

SelectField::SelectField(std::string rbg_window){
    /**
    * The values for the dst_points_ vector are fixed, and were calculated considering the dimensions of the RGB and
    * depth images given by the kinect.
    */
    dst_points_.push_back(cv::Point2f(0.0,0.0));
    dst_points_.push_back(cv::Point2f(320.0,0.0));
    dst_points_.push_back(cv::Point2f(640.0,0.0));

    dst_points_.push_back(cv::Point2f(0.0,480.0));
    dst_points_.push_back(cv::Point2f(320.0,480.0));
    dst_points_.push_back(cv::Point2f(640.0,480.0));

    rbg_cap_.fromWindow(rbg_window);
}

void SelectField::run(){
    auto src_points = rbg_cap_.getVector();

    if (src_points.size() != 6)
    {
        ROS_WARN("6 points are needed for homography.");
        return;
    }

    cv::Mat srcp(src_points);
    cv::Mat dstp(dst_points_);

    field_matrix_ = cv::findHomography(srcp,dstp);
}