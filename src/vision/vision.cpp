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

/**
 * Starts the vision system.
 */
Vision::Vision()
{
    has_received_all_images_ = has_received_first_rgb_image_ =
        has_received_first_depth_image_ = false;
    ConfigParser::getInstance().parseConfigFile();
    Segmenter::getInstance().init();
    Identifier::getInstance().init();
}

/**
 * Execute vision processing. Nothing will be done until the first image has been
 * received.
 */
void Vision::run()
{
    if (has_received_first_rgb_image_)
    {
        Segmenter::getInstance().runSegmentationAlgorithms();
        Identifier::getInstance().runIdentificationAlgorithms();
        Tracker::getInstance().runTracking();

        VisionGUI::getInstance().showOutputImages();
    }
}

/**
 * Receives the raw rgb image, and passes it on to the RawImage class and the VisionGUI.
 */
void Vision::setRawRGBImage(const cv::Mat &rgb_image)
{
    if (isValidSize(rgb_image))
    {
        has_received_first_rgb_image_ = true;
        RawImage::getInstance().setRawRGBImage(rgb_image);
        VisionGUI::getInstance().setInitialRGBImage(rgb_image);
    }

    if (has_received_first_rgb_image_ and has_received_first_depth_image_)
        has_received_all_images_ = true;
}

/**
 * Receives the raw depth image, and passes it on to the RawImage class and the VisionGUI.
 */
void Vision::setRawDepthImage(const cv::Mat &depth_image)
{
    if (isValidSize(depth_image))
    {
        has_received_first_depth_image_ = true;
        RawImage::getInstance().setRawDepthImage(depth_image);
        VisionGUI::getInstance().setInitialDepthImage(depth_image);
    }

    if (has_received_first_rgb_image_ and has_received_first_depth_image_)
        has_received_all_images_ = true;
}

/**
 * Check whether an image has a valid size, i.e., neither the width nor the height can be zero.
 * @return true if the image has proper size, false otherwise.
 */
bool Vision::isValidSize(const cv::Mat &img)
{
    return not (img.rows == 0 or img.cols == 0);
}

std::unordered_map<std::string, TrackingOutput> Vision::getVisionOutput()
{
    return Tracker::getInstance().getTrackerOutput();
}
