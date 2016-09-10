#ifndef VISION_CAPTURE_POINTS_H_
#define VISION_CAPTURE_POINTS_H_

#include <opencv2/opencv.hpp>
#include <string>
#include <ros/ros.h>

class CapturePoints
{
  public:
    CapturePoints(){};
    CapturePoints(std::string window);
    void fromWindow(std::string window);
    std::vector<cv::Point2f> getVector();
    int getVectorSize();

  private:
    static void mouseCallback(int event, int x, int y, int, void*);
    
    std::vector<cv::Point2f> points_;
};


#endif