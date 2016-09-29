#include "calibration/matching.hpp"

Matching::Matching(std::string rgb_window, std::string depth_window){
    depth_cap_.fromWindow(depth_window);
    rbg_cap_.fromWindow(rgb_window);
    rgb_window_ = rgb_window;
    depth_window_ = depth_window;
    
    createWindows(rgb_window_, depth_window_);
    
    bool calibrate;
    ros::param::get("/vision/calibration/calibrate_match_matrix", calibrate);
    
    //if needs calibration
    if (calibrate == true)
        is_done_ = false;
    else //retrieve matching matrix
    {
        is_done_ = true;
        FileManager file("match", "read");
        matching_matrix_ = file.read();
        close();
    }
    
}

void Matching::run(){
    rgb_points_ = rbg_cap_.getVector();
    depth_points_ = depth_cap_.getVector();

    if (rgb_points_.size() != 6 or depth_points_.size() != 6)
    {
        ROS_WARN("6 points are needed for calibration.");
        return;
    }

    close();
    cv::Mat srcp(rgb_points_);
    cv::Mat dstp(depth_points_);

    matching_matrix_ = cv::findHomography(srcp,dstp);
    is_done_ = true;
    ROS_WARN("DONE");

    //write matching matrix to file
    FileManager file("match", "write");    
    file.write(matching_matrix_);
}

cv::Mat Matching::getMatrix(){
    return matching_matrix_;
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
}

cv::Mat Matching::match(cv::Mat frame){
    cv::Mat result;
    
    if(matching_matrix_.rows > 0 and matching_matrix_.cols > 0)
        cv::warpPerspective(frame, result, matching_matrix_, cv::Size(640,480));
    else
        ROS_ERROR("Matching matrix not calculated");
    return result;
}

void Matching::createWindows(std::string rgb_name, std::string depth_name){
    cv::namedWindow(rgb_name);
    cv::namedWindow(depth_name);
}