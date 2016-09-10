#ifndef VISION_CAPTURE_POINTS
#define VISION_CAPTURE_POINTS

#include <opencv2/opencv.hpp>
#include <string>
#include <ros/ros.h>

class CapturePoints
{
  public:
	CapturePoints();
	~CapturePoints();
	void fromImage(cv::Mat, std::string);



	static void mouseCallback(int event, int x, int y, int, void*);

  private:
    cv::Mat frame_;

  	std::string rgb_frame_title_;

    std::vector<cv::Point2f> points_; // Points for rgb homography
};


#endif