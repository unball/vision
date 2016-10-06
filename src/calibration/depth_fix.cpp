#include "calibration/depth_fix.hpp"

DepthFix::DepthFix(){
}

cv::Mat DepthFix::fix(cv::Mat &depth_frame){
      
    double min;
    double max;
    cv::minMaxIdx(depth_frame, &min, &max);

    // this is great. It converts your grayscale image into a tone-mapped one,
    // much more pleasing for the eye
    // function is found in contrib module, so include contrib.hpp
    // and link accordingly
    cv::Mat falseColorsMap;
    cv::normalize(depth_frame, falseColorsMap, 0, 256, cv::NORM_MINMAX, CV_8UC1);
    return falseColorsMap;
}

void DepthFix::adjustNoise(cv::Mat &image){
    cv::createTrackbar("Noise thresh", "Depth Calibration matching", &noise_thresh_, 255);
    cv::createTrackbar("Noise thresh", "Calibrated Depth frame", &noise_thresh_, 255);

    for (int i = 0; i < image.rows; ++i)
        for (int j = 0; j < image.cols; ++j)
            if (image.at<uchar>(i, j) <= noise_thresh_)
                image.at<uchar>(i, j) = 0;
}
