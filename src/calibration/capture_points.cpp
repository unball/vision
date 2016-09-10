#include "calibration/capture_points.hpp"

CapturePoints::CapturePoints(std::string window){
    fromWindow(window);
}

void CapturePoints::fromWindow(std::string window){
    cv::setMouseCallback(window, mouseCallback, (void*)&points_);
}

void CapturePoints::mouseCallback(int event, int x, int y, int, void* points_vector){
    auto *point_ptr = (std::vector<cv::Point2f>*) points_vector;
    if(event == cv::EVENT_LBUTTONDOWN){
        ROS_INFO("RGB frame button click at: (%d,%d)", x, y);
        point_ptr->push_back(cv::Point2f(x, y));
    }
}

std::vector<cv::Point2f> CapturePoints::getVector(){
    return points_;
}

int CapturePoints::getVectorSize(){
    return points_.size();
}