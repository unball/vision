/**
 * @file   vision.hpp
 * @author Gabriel Naves da Silva
 * @date   27/02/2016
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief  Vision class
 *
 * Defines computer vision class
 */

#ifndef VISION_VISION_H_
#define VISION_VISION_H_

#include <ros/ros.h>

#include <opencv2/opencv.hpp>

#include <vision/segmenter.hpp>
#include <vision/identifier.hpp>
#include <vision/tracker.hpp>
#include <vision/tracking_output.hpp>
#include <vision/config_parser.hpp>
#include <vision/vision_gui.hpp>
#include <vision/raw_image.hpp>

class Vision
{
  public:
    static Vision& getInstance();

    void run();
    void setRawRGBImage(const cv::Mat &rgb_image);
    void setRawDepthImage(const cv::Mat &depth_image);
    
    std::unordered_map<std::string, TrackingOutput> getVisionOutput();


    std::unordered_map<std::string, TrackingOutput> getVisionOutput();

  private:
    Vision();

    bool isValidSize(const cv::Mat &img);

    bool has_received_all_images_;
    bool has_received_first_rgb_image_;
    bool has_received_first_depth_image_;
};

#endif // VISION_VISION_H_