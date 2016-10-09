#ifndef VISION_IDENTIFICATION_OUTPUT_H_
#define VISION_IDENTIFICATION_OUTPUT_H_

#include <opencv2/opencv.hpp>

struct IdentificationOutput
{
    /* Put any information needed here. */
    std::vector<cv::Point2f> object_pose;
};

#endif // VISION_IDENTIFICATION_OUTPUT_H_
