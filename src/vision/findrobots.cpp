#include <vision/findrobots.hpp>

REGISTER_ALGORITHM_DEF(FindRobots);

void FindRobots::run(){
    RawImage::getInstance().getRawDepthImage().copyTo(input_);
    auto upperPoints = preProcessor(input_);
    cv::imshow(window_name_, upperPoints);
    if(cv::waitKey(30) == 'r')
        cv::destroyWindow(window_name_);
}

void FindRobots::init(){
    cv::namedWindow(window_name_);
    cv::createTrackbar("Canny Thresh 1", window_name_, &cannythresh1_, 100);
    cv::createTrackbar("Canny Thresh 2", window_name_, &cannythresh2_, 300);

}

cv::Mat FindRobots::preProcessor(cv::Mat input){
    cv::Mat cannyMat;
    std::vector<cv::Mat> channels(3);

    input.copyTo(cannyMat);
    
    cv::cvtColor(cannyMat, cannyMat, CV_BGR2HSV);
    cv::split(cannyMat, channels);
    
    cv::Canny(channels[2], cannyMat, cannythresh1_, cannythresh2_, 3);
    return cannyMat;
}
