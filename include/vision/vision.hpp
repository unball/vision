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
    void setRawImage(const cv::Mat &image);

    std::unordered_map<std::string, TrackingOutput> getVisionOutput();

  private:
    Vision();

    bool isValidSize(const cv::Mat &img);

    bool has_received_first_image_;
};

#endif // VISION_VISION_H_
