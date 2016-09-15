#include "calibration/select_field.hpp"

SelectField::SelectField(std::string rgb_window){
    /**
    * The values for the dst_points_ vector are fixed, and were calculated considering the dimensions of the RGB and
    * depth images given by the kinect.
    */
    rgb_window_ = rgb_window;

    dst_points_.push_back(cv::Point2f(0.0,0.0));
    dst_points_.push_back(cv::Point2f(320.0,0.0));
    dst_points_.push_back(cv::Point2f(640.0,0.0));

    dst_points_.push_back(cv::Point2f(0.0,480.0));
    dst_points_.push_back(cv::Point2f(320.0,480.0));
    dst_points_.push_back(cv::Point2f(640.0,480.0));
    is_selected_ = false;
}

void SelectField::close(){
    cv::destroyWindow(rgb_window_);
}

void SelectField::run(){
    auto src_points = rgb_cap_.getVector();

    if (src_points.size() != 6)
    {
        ROS_WARN("6 points are needed for homography.");
        return;
    }
    
    close();

    cv::Mat srcp(src_points);
    cv::Mat dstp(dst_points_);

    field_matrix_ = cv::findHomography(srcp,dstp);
    is_selected_ = true;
}

void SelectField::start(){
    cv::namedWindow(rgb_window_);
    cv::waitKey(1);
    rgb_cap_.fromWindow(rgb_window_);
}

void SelectField::showFrame(cv::Mat rgb_frame){
     if (not (rgb_frame.rows == 0 or rgb_frame.cols == 0))
        cv::imshow(rgb_window_, rgb_frame);

    cv::waitKey(1);
}

bool SelectField::isDone(){
    return is_selected_;
}

cv::Mat SelectField::warp(cv::Mat rgb_frame){
     cv::Mat result;
    
    if(field_matrix_.rows != 0 or field_matrix_.cols != 0)
        cv::warpPerspective(rgb_frame, result, field_matrix_, cv::Size(640,480));
    else
        ROS_ERROR("Matching matrix not calculated");
    return result;
}