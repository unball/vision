#ifndef VISION_TRACKING_OUTPUT_H_
#define VISION_TRACKING_OUTPUT_H_

#include <opencv2/opencv.hpp>

struct TrackingOutput
{
    std::vector<cv::Point2f> positions;
    std::vector<float> orientations;
};

#endif // VISION_TRACKING_OUTPUT_H_
