#ifndef VISION_IDENTIFICATION_OUTPUT_H_
#define VISION_IDENTIFICATION_OUTPUT_H_

#include <opencv2/opencv.hpp>

struct IdentificationOutput
{
    /* Put any information needed here. */

    cv::Point2f object_center;
};

#endif // VISION_IDENTIFICATION_OUTPUT_H_
