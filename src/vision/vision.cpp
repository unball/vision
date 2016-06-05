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
    has_received_first_image_ = false;
}

/**
 * Execute vision processing.
 */
void Vision::run()
{
    if (has_received_first_image_)
    {
        Segmenter::getInstance().runSegmentationAlgorithms();
        Identifier::getInstance().runIdentificationAlgorithms();
        Tracker::getInstance().runTracking();

        VisionGUI::getInstance().showOutputImage();
    }
}

void Vision::setRawImage(const cv::Mat &raw_image)
{
    has_received_first_image_ = true;
    RawImage::getInstance().setRawImage(raw_image);
    VisionGUI::getInstance().setInitialImage(raw_image);
}
