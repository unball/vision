#include "vision/robot_identification.hpp"

REGISTER_ALGORITHM_DEF(RobotIdentification);

void RobotIdentification::run(){
    mask_ = segmentation_algorithm_->getSegmentationRGBOutput();
    cv::Mat mask = mask_;
    rgb_img_ = VisionGUI::getInstance().getOutputRGBImage();
    cv::Mat rgb_img = rgb_img_;

    find(mask, rgb_img);

    output_info_->object_pose = robots_coord_;
    output_info_->object_orientation = robots_orientation_;
}

void RobotIdentification::init(){
    robots_coord_ = std::vector<cv::Point2f>(3);
    robots_orientation_ = std::vector<float>(3);
    window_name_ = segmentation_algorithm_->getFullName();
    cv::namedWindow(window_name_);
    cv::createTrackbar("Area", window_name_, &area_, 2000);
    output_info_ = std::make_shared<IdentificationOutput>();

    auto sourceDir = ros::package::getPath("vision").append("/data/");
    auto filename = "color_calibration.yaml";

    color_reader_ = cv::FileStorage(sourceDir+filename,cv::FileStorage::READ);
    if (color_reader_.isOpened())
    {
        color_reader_["Red"] >> red_mat_;
        color_reader_["Pink"] >> pink_mat_;
        color_reader_["Green"] >> green_mat_;
    }

}

void RobotIdentification::find(cv::Mat input, cv::Mat rgb_input){
    cv::Mat mask;
    input.copyTo(mask);
    ROS_INFO("FIND");

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

    ROS_INFO("CONTOURS");

    int orientation_index;
    auto color = cv::Scalar::all(255);
    for (uint i = 0, robot_index = 0; i < newcontours.size() && robot_index < 3; ++i, robot_index++)
    {
        auto newboundingRect = cv::boundingRect(newcontours[i]);

        if (newboundingRect.area() > area_)
        {   
            cv::Mat roi;
            cv::Mat mask;
            rgb_input(newboundingRect).copyTo(roi);
            mask = identify(newcontours[i], &orientation_index, robot_index, roi, input, newboundingRect);
            ROS_INFO("index: %d", orientation_index);
            input(newboundingRect).copyTo(roi);
            findOrientation(roi, orientation_index, mask);
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


cv::Mat RobotIdentification::identify(std::vector<cv::Point> contour, int *orientation_index, int index, cv::Mat roi, cv::Mat input, cv::Rect boundingRect){
    cv::Moments moments;
    moments = cv::moments(contour, true);
    auto robot_pose = cv::Point2f(moments.m10/moments.m00, moments.m01/moments.m00);
    ROS_INFO("IDENTIFY");
    int orient_aux = *orientation_index;
    if (arguments_ == "ours")
    {   
        ROS_INFO("OURS");
        //ROS_ERROR("our robots");
        cv::cvtColor(roi, roi, CV_BGR2HSV);

        cv::Mat red_mask, pink_mask, green_mask;

        cv::inRange(roi,
                    cv::Scalar(red_mat_.at<int>(0,0), red_mat_.at<int>(0,1), red_mat_.at<int>(0,2)),
                    cv::Scalar(red_mat_.at<int>(1,0), red_mat_.at<int>(1,1), red_mat_.at<int>(1,2)),
                    red_mask);
        cv::inRange(roi,
                    cv::Scalar(pink_mat_.at<int>(0,0), pink_mat_.at<int>(0,1), pink_mat_.at<int>(0,2)),
                    cv::Scalar(pink_mat_.at<int>(1,0), pink_mat_.at<int>(1,1), pink_mat_.at<int>(1,2)),
                    pink_mask);
        cv::inRange(roi,
                    cv::Scalar(green_mat_.at<int>(0,0), green_mat_.at<int>(0,1), green_mat_.at<int>(0,2)),
                    cv::Scalar(green_mat_.at<int>(1,0), green_mat_.at<int>(1,1), green_mat_.at<int>(1,2)),
                    green_mask);

        bool isRed = false, isPink = false, isGreen = false;

        ROS_INFO("IN RANGE");

       isRed = robotColor(red_mask);
       isPink = robotColor(pink_mask);
       isGreen = robotColor(green_mask);
       ROS_INFO("ROBOT");


       cv::Mat mask;
       cv::Mat mask_inverted;
       ROS_INFO("MASK");
       input(boundingRect).copyTo(mask);
       cv::bitwise_not(mask, mask_inverted);
        ROS_INFO("BITWISE");
        
        if(isRed){
            ROS_INFO("RED");
            robots_coord_[0] = robot_pose;
            cv::bitwise_and(red_mask, mask_inverted, roi);
            *orientation_index = 0;
            return roi;
        }
        else if(isPink){
            ROS_INFO("PINK");

            robots_coord_[1] = robot_pose;
            cv::bitwise_and(pink_mask, mask_inverted, roi);
            *orientation_index = 1;
            return roi;
        }
        else if(isGreen){
            ROS_INFO("GREEN");

            robots_coord_[2] = robot_pose;
            cv::bitwise_and(green_mask, mask_inverted, roi);
            *orientation_index = 2;
            return roi;
        }else{
            return mask;
        }

    }
    else if (arguments_ == "theirs"){
        ROS_INFO("THEIRS");
        robots_coord_[index] = robot_pose;
        return roi;
    }
    
}

bool RobotIdentification::robotColor(cv::Mat mask){

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;

    cv::findContours(mask, contours, hierarchy, CV_RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    ROS_INFO("ROBOT COLOR");
    for(int i = 0, sum_ = 0; i < contours.size(); i++){
        auto bounding_rect = cv::boundingRect(contours[i]);
        sum_ += bounding_rect.area();
        if (sum_ > 10)
            return true;
    }
    
    return false;
}


void RobotIdentification::findOrientation(cv::Mat mask, int index, cv::Mat orient_circle){
    std::vector<cv::Vec4i> hierarchy;
    std::vector<std::vector<cv::Point>> contours;
    
    cv::Moments moments;
    cv::Point2f robot_center;
    cv::Point2f robot_id;

    ROS_INFO("22222");
    float area = 0;
    cv::findContours(mask, contours, hierarchy, CV_RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    ROS_INFO("33333");
    for (uint i = 0; i < contours.size(); ++i)
    {
        auto newboundingRect = cv::boundingRect(contours[i]);

        if (newboundingRect.area() > area)
        {
            moments = cv::moments(contours[i], true);
            robot_center = cv::Point2f(moments.m10/moments.m00, moments.m01/moments.m00);
            area = newboundingRect.area();
        }
    }
    ROS_INFO("44444");
    cv::findContours(orient_circle, contours, hierarchy, CV_RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    area = 100000.0;
    for (uint i = 0; i < contours.size(); ++i)
    {
        auto newboundingRect = cv::boundingRect(contours[i]);

        if (newboundingRect.area() < area)
        {
            moments = cv::moments(contours[i], true);
            robot_id = cv::Point2f(moments.m10/moments.m00, moments.m01/moments.m00);
            area = newboundingRect.area();
        }
    }
    ROS_INFO("55555");
     
    auto orientation_vector = robot_id - robot_center;
    auto theta = atan2(orientation_vector.y, orientation_vector.x);

    // cv::Point2f orient = cv::Point2f(robot_center.x + 10*cos(theta), robot_center.y + 10*sin(theta));
    // cv::circle(mask, robot_center, 2, cv::Scalar(150,0,0));
    // cv::circle(mask, robot_id, 2, cv::Scalar(150,0,0));
    // cv::line(mask, robot_center, orient, cv::Scalar(133,255,20));
    // cv::imshow("mask", mask);

    robots_orientation_[index] = theta;

}