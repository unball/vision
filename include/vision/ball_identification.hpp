#ifndef BALL_IDENTIFICATION_H
#define BALL_IDENTIFICATION_H 

#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <vision/algorithm_factory.hpp>
#include <vision/identification_algorithm.hpp>
#include <vision/raw_image.hpp>
#include <vision/vision_gui.hpp>

class BallIdentification : public IdentificationAlgorithm
{
  public:
    ALGORITHM_TYPE(BallIdentification);
    void run();
    void init();
  private:
    CvSeq* findLargerBlob(CvSeq* contours);
    cv::Point2f calcBallPose(CvSeq* contours);
    REGISTER_ALGORITHM_DEC(BallIdentification);
    cv::Mat rgb_segmented_;
    IplImage rgb_segmented_ipl_;
    CvMemStorage* storage;
    CvMemStorage *mem;
    CvSeq *contours;
    
};


#endif