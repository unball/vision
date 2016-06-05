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

#include <vector>

#include <ros/ros.h>

#include <opencv2/opencv.hpp>

#include <vision/segmenter.hpp>
#include <vision/identifier.hpp>
#include <vision/tracker.hpp>
#include <vision/config_parser.hpp>
#include <vision/vision_gui.hpp>

class Vision
{
  public:
    static Vision& getInstance();

    void run();

    void setRawImage(const cv::Mat &raw_image);
    bool isValidSize(const cv::Mat &img);

  private:
    Vision();

    bool has_received_first_image_;
};

#endif // VISION_VISION_H_
