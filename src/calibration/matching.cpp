#include "calibration/matching.hpp"

Matching::Matching(std::string rgb_window, std::string depth_window){
    depth_cap_.fromWindow(depth_window);
    rbg_cap_.fromWindow(rgb_window);
}

void Matching::run(){
    auto rgb_points = rbg_cap_.getVector();
    auto depth_points = depth_cap_.getVector();
    
    if (rgb_points.size() != 6 or depth_points.size() != 6)
    {
        ROS_WARN("6 points are needed for calibration.");
        return;
    }

    cv::Mat srcp(rgb_points);
    cv::Mat dstp(depth_points);

    matching_matrix_ = cv::findHomography(srcp,dstp);
}

cv::Mat Matching::getMatrix(){
    return matching_matrix_;
}