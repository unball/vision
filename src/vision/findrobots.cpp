#include "findrobots.hpp"

FindRobots::FindRobots(){
}

void FindRobots::find(cv::Mat input){
    auto upperPoints = preProcessor(input);
    cv::imshow("Canny", upperPoints);
}

cv::Mat FindRobots::preProcessor(cv::Mat input){
    cv::Mat cannyMat;
    std::vector<cv::Mat> channels(3);

    input.copyTo(cannyMat);
    
    cv::cvtColor(cannyMat, cannyMat, CV_BGR2HSV);
    cv::split(cannyMat, channels);
    
    cv::Canny(channels[2], cannyMat, _cannythresh1, _cannythresh2, 3);
    cannyMat = derivate(cannyMat);
    return cannyMat;
}

void FindRobots::extractPoints(cv::Mat& upperPoints){
    auto color = cv::Scalar::all(0);
    cv::drawContours(upperPoints, params_.contours, params_.largestAreaIndex, color, 6, 10, params_.hierarchy);
    int const max_kernel_size = 21;

    cv::createTrackbar( "Kernel size:\n 2n +1", "RGB Video", &erosion_size, max_kernel_size);
    cv::Mat element = cv::getStructuringElement( cv::MORPH_RECT,
                                       cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                       cv::Point( erosion_size, erosion_size ) );
    cv::createTrackbar( "Kernel size 2:\n 2n +1", "RGB Video", &erosion_size2, max_kernel_size);

    cv::Mat element2 = getStructuringElement( cv::MORPH_RECT,
                                       cv::Size( 2*erosion_size2 + 1, 2*erosion_size2+1 ),
                                       cv::Point( erosion_size2, erosion_size2 ) );
    cv::dilate(upperPoints, upperPoints, element);
    cv::erode(upperPoints, upperPoints, element2);
}