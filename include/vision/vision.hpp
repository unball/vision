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

#ifndef UNBALL_VISION_H_
#define UNBALL_VISION_H_

#include <vector>

#include <ros/ros.h>

#include <opencv2/opencv.hpp>

#include <vision/segmenter.hpp>
#include <vision/identifier.hpp>
#include <vision/tracker.hpp>
#include <vision/config_parser.hpp>

class Vision
{
  public:
    static Vision& getInstance();

    void run();

    void setRawImage(const cv::Mat &raw_image);

  private:
    Vision();
};

#endif // UNBALL_VISION_H_
