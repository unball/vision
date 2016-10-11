#include "vision/robot_identification.hpp"

REGISTER_ALGORITHM_DEF(RobotIdentification);

void RobotIdentification::run(){
    mask_ = segmentation_algorithm_->getSegmentationRGBOutput();
    cv::Mat mask = mask_;
    
    find(mask);

    output_info_->object_pose = robots_coord_;
    output_info_->object_orientation = robots_orientation_;
}

void RobotIdentification::init(){
    robots_coord_ = std::vector<cv::Point2f>(3);
    robots_orientation_ = std::vector<cv::Point2f>(3);
    window_name_ = segmentation_algorithm_->getFullName();
    cv::namedWindow(window_name_);
    cv::createTrackbar("Area", window_name_, &area_, 2000);
    output_info_ = std::make_shared<IdentificationOutput>();

}

void RobotIdentification::find(cv::Mat input){
    cv::Mat mask;
    input.copyTo(mask);

    std::vector<std::vector<cv::Point>> newcontours;
    std::vector<cv::Vec4i> newhierarchy;
    uchar *p;
    cv::dilate(mask, mask, cv::Mat());

    for (int i = 0; i < mask.rows; ++i)
    {   

        p = mask.ptr<uchar>(i);
        for (int j = 0; j < mask.cols; ++j)
        {
            if ((int)p[j] == 255)
            {
                int area = 10;
                bool islost = true;
                
                for (int k = 0; k < 20 and j+k < mask.cols; ++k)
                    if(p[j+k] == 255)
                        islost = false;

                if (islost)
                    p[j] = 0;

                for (int k = 5; k < area and p[j+1] != 255; ++k, ++j)
                    p[j] = 255;
            }else if ((int)p[j] != 0)
            {
                p[j] = 0;
            }
        }
    }
    
    cv::findContours(mask, newcontours, newhierarchy, CV_RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    auto color = cv::Scalar::all(255);
    for (uint i = 0, robot_index = 0; i < newcontours.size() && robot_index < 3; ++i, robot_index++)
    {
        auto newboundingRect = cv::boundingRect(newcontours[i]);

        if (newboundingRect.area() > area_)
        {
            identify(newcontours[i], robot_index);
            cv::Mat roi;
            input(newboundingRect).copyTo(roi);
            findOrientation(roi, robot_index);
        }
    }
    if (not hasclosed_){
        {
            cv::imshow(window_name_, input);
            if (cv::waitKey(30) == 'w'){
                cv::destroyWindow(window_name_);
                hasclosed_ = true;  
            }
        } 
    }
}


void RobotIdentification::identify(std::vector<cv::Point> contour, int index){
    cv::Moments moments;
    moments = cv::moments(contour, true);
    auto robot_pose = cv::Point2f(moments.m10/moments.m00, moments.m01/moments.m00);

    robots_coord_[index] = robot_pose;
}

void RobotIdentification::findOrientation(cv::Mat mask, int index){
    cv::Mat original;
    mask.copyTo(original);
    std::vector<cv::Vec4i> hierarchy;
    std::vector<std::vector<cv::Point>> contours;
    cv::Mat mask_inverted = cv::Mat();
    
    cv::Moments moments;
    cv::Point2f robot_center;
    cv::Point2f robot_id;
    
    cv::bitwise_not(mask, mask_inverted);
    
    float area = 0;
    cv::findContours(mask, contours, hierarchy, CV_RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for (uint i = 0, robot_index = 0; i < contours.size() && robot_index < 3; ++i, robot_index++)
    {
        auto newboundingRect = cv::boundingRect(contours[i]);

        if (newboundingRect.area() > area)
        {
            moments = cv::moments(contours[i], true);
            robot_center = cv::Point2f(moments.m10/moments.m00, moments.m01/moments.m00);
            area = newboundingRect.area();
        }
    }
    
    cv::findContours(mask_inverted, contours, hierarchy, CV_RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    area = 10000.0;
    for (uint i = 0, robot_index = 0; i < contours.size() && robot_index < 3; ++i, robot_index++)
    {
        auto newboundingRect = cv::boundingRect(contours[i]);

        if (newboundingRect.area() < area)
        {
            moments = cv::moments(contours[i], true);
            robot_id = cv::Point2f(moments.m10/moments.m00, moments.m01/moments.m00);
            area = newboundingRect.area();
        }
    }
     
    auto orientation_vector = robot_id - robot_center;
    robots_orientation_[index] = orientation_vector;
}