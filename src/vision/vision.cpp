/**
 * @file   vision.cpp
 * @author Gabriel Naves da Silva
 * @date   27/02/2016
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Vision class
 */

#include <vision/vision.hpp>

Vision& Vision::getInstance()
{
    static Vision vision;
    return vision;
}

Vision::Vision()
{
    ConfigParser::getInstance().parseConfigFile();
}

/**
 * Execute vision processing.
 */
void Vision::run()
{
    Segmenter::getInstance().runSegmentationAlgorithms();
    Identifier::getInstance().runIdentificationAlgorithms();
    Tracker::getInstance().runTracking();
}

void Vision::setRawImage(const cv::Mat &raw_image)
{
    RawImage::getInstance().setRawImage(raw_image);
}
