#ifndef IDENTIFICATION_H
#define IDENTIFICATION_H

#include <ros/ros.h>
#include <ros/package.h>
#include <opencv2/opencv.hpp>
#include <vision/algorithm_factory.hpp>
#include <vision/identification_algorithm.hpp>
#include <vision/raw_image.hpp>
#include <vision/vision_gui.hpp>

#include <string>
#include <memory>
#include <cmath>
#include <cstdio>

class RobotIdentification : public IdentificationAlgorithm
{
public:
    ALGORITHM_TYPE(RobotIdentification);
    void run();
    void init();

private:
    REGISTER_ALGORITHM_DEC(RobotIdentification);
    void find(cv::Mat input, cv::Mat rgb_input);
    cv::Mat identify(std::vector<cv::Point> contour, int &orientation_index, int index, cv::Mat roi, cv::Mat input, cv::Rect boundingRect);
    void findOrientation(cv::Mat mask, int index, cv::Mat orient_circle);
    bool robotColor(cv::Mat mask);

    cv::FileStorage color_reader_;
    cv::Mat red_mat_, pink_mat_, green_mat_;
    
    std::vector<cv::Rect> robots_;
    std::vector<cv::Point2f> robots_coord_;
    std::vector<float> robots_orientation_;
    cv::Mat mask_;
    cv::Mat rgb_img_;

    std::vector<cv::Vec4i> hierarchy_;

    std::string window_name_;

    std::string allies_;
    std::string enemies_;

    int area_ = 1155;
    bool hasclosed_; 

};
#endif