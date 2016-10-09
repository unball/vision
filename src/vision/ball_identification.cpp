#include <vision/ball_identification.hpp>

REGISTER_ALGORITHM_DEF(BallIdentification);

void BallIdentification::init(){
    storage = cvCreateMemStorage(0);
    mem = cvCreateMemStorage(0);
    contours = 0;
    output_info_ = std::make_shared<IdentificationOutput>();
    output_info_->object_pose = std::vector<cv::Point2f>(1);
}

void BallIdentification::run(){
    rgb_segmented_ = segmentation_algorithm_->getSegmentationRGBOutput();
    
    rgb_segmented_ipl_ = rgb_segmented_;
    cvFindContours(&rgb_segmented_ipl_, mem, &contours, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0));
    
    CvSeq* largestContour = findLargerBlob(contours);
    auto ball_pose = calcBallPose(largestContour);
    output_info_->object_pose[0] = ball_pose;
    
}

CvSeq* BallIdentification::findLargerBlob(CvSeq* contours){

    double largestArea = 0;                    //Const. for the largest area
    CvSeq* largest_contour = NULL;             //Contour for the largest area
    while (contours != NULL){           //If the current contour available
        double area = fabs(cvContourArea(contours,CV_WHOLE_SEQ, false));   //Get the current contour's area as "area"    
        if(area > largestArea){            //If "area" is larger than the previous largest area
            largestArea = area;
            largest_contour = contours; 
        }
        contours = contours->h_next;  //Search for the next contour
    }
    return largest_contour;
}

cv::Point2f BallIdentification::calcBallPose(CvSeq* contour){
    if (contour != NULL)
    {
        CvMoments moments;
        cvMoments(contour, &moments);
        cv::Point2f ballPose =  cv::Point2f(moments.m10/moments.m00, moments.m01/moments.m00);
        return ballPose;

    }
    return cv::Point2f(-1.0, -1.0);
}