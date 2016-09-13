#include "calibration/matching.hpp"

Matching::Matching(std::string rgb_window, std::string depth_window){
    depth_cap_.fromWindow(depth_window);
    rbg_cap_.fromWindow(rgb_window);
<<<<<<< HEAD
    rgb_window_ = rgb_window;
    depth_window_ = depth_window;
    is_done_ = false;
}

void Matching::run(){
    rgb_points_ = rbg_cap_.getVector();
    depth_points_ = depth_cap_.getVector();
    

    if (rgb_points_.size() != 6 or depth_points_.size() != 6)
=======
}

void Matching::run(){
    auto rgb_points = rbg_cap_.getVector();
    auto depth_points = depth_cap_.getVector();
    
    if (rgb_points.size() != 6 or depth_points.size() != 6)
>>>>>>> d5fb4c05583d0a4dac4a3e500e8b834144c29a3f
    {
        ROS_WARN("6 points are needed for calibration.");
        return;
    }

<<<<<<< HEAD
    close();
    cv::Mat srcp(rgb_points_);
    cv::Mat dstp(depth_points_);

    matching_matrix_ = cv::findHomography(srcp,dstp);
    is_done_ = true;
=======
    cv::Mat srcp(rgb_points);
    cv::Mat dstp(depth_points);

    matching_matrix_ = cv::findHomography(srcp,dstp);
>>>>>>> d5fb4c05583d0a4dac4a3e500e8b834144c29a3f
}

cv::Mat Matching::getMatrix(){
    return matching_matrix_;
<<<<<<< HEAD
}

bool Matching::isDone(){
    return is_done_;
}

void Matching::close(){
    cv::destroyWindow(rgb_window_);
    cv::destroyWindow(depth_window_);
}

void Matching::showFrames(cv::Mat rgb_frame, cv::Mat depth_frame){
     if (not (rgb_frame.rows == 0 or rgb_frame.cols == 0))
        cv::imshow(rgb_window_, rgb_frame);

    if (not (depth_frame.rows == 0 or depth_frame.cols == 0))
        cv::imshow(depth_window_, depth_frame);

    cv::waitKey(1);
=======
>>>>>>> d5fb4c05583d0a4dac4a3e500e8b834144c29a3f
}