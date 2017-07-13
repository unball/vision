#include "vision/robot_identification.hpp"

REGISTER_ALGORITHM_DEF(RobotIdentification);

void RobotIdentification::run(){


    mask_ = segmentation_algorithm_->getSegmentationOutput();
    cv::Mat mask = mask_;
    rgb_img_ = VisionGUI::getInstance().getOutputImage();
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
        color_reader_["Robot0"] >> robot0_mat_;
        color_reader_["Robot1"] >> robot1_mat_;
        color_reader_["Robot2"] >> robot2_mat_;
    }

}

void RobotIdentification::find(cv::Mat input, cv::Mat rgb_input){
    cv::Mat mask;
    input.copyTo(mask);

    std::vector<std::vector<cv::Point>> newcontours;
    std::vector<cv::Vec4i> newhierarchy;
    uchar *p;
    cv::dilate(mask, mask, cv::Mat());

    // for (int i = 0; i < mask.rows; ++i)
    // {   

    //     p = mask.ptr<uchar>(i);
    //     for (int j = 0; j < mask.cols; ++j)
    //     {
    //         if ((int)p[j] == 255)
    //         {
    //             int area = 10;
    //             bool islost = true;
                
    //             for (int k = 0; k < 20 and j+k < mask.cols; ++k)
    //                 if(p[j+k] == 255)
    //                     islost = false;

    //             if (islost)
    //                 p[j] = 0;

    //             for (int k = 5; k < area and p[j+1] != 255; ++k, ++j)
    //                 p[j] = 255;
    //         }else if ((int)p[j] != 0)
    //         {
    //             p[j] = 0;
    //         }
    //     }
    // }
    
    cv::findContours(mask, newcontours, newhierarchy, CV_RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

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
            input(newboundingRect).copyTo(roi);
            findOrientation(roi, orientation_index, mask);
        }
        else{
            robots_coord_[robot_index] = cv::Point2f(0, 0);
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
    int orient_aux = *orientation_index;
    if (arguments_ == "ours")
    {   
        cv::cvtColor(roi, roi, CV_BGR2HSV);

        cv::Mat robot0_mask, robot1_mask, robot2_mask;

        cv::inRange(roi,
                    cv::Scalar(robot0_mat_.at<int>(0,0), robot0_mat_.at<int>(0,1), robot0_mat_.at<int>(0,2)),
                    cv::Scalar(robot0_mat_.at<int>(1,0), robot0_mat_.at<int>(1,1), robot0_mat_.at<int>(1,2)),
                    robot0_mask);
        cv::inRange(roi,
                    cv::Scalar(robot1_mat_.at<int>(0,0), robot1_mat_.at<int>(0,1), robot1_mat_.at<int>(0,2)),
                    cv::Scalar(robot1_mat_.at<int>(1,0), robot1_mat_.at<int>(1,1), robot1_mat_.at<int>(1,2)),
                    robot1_mask);
        cv::inRange(roi,
                    cv::Scalar(robot2_mat_.at<int>(0,0), robot2_mat_.at<int>(0,1), robot2_mat_.at<int>(0,2)),
                    cv::Scalar(robot2_mat_.at<int>(1,0), robot2_mat_.at<int>(1,1), robot2_mat_.at<int>(1,2)),
                    robot2_mask);

        bool isRobot0 = false, isRobot1 = false, isRobot2 = false;


        isRobot0 = robotColor(robot0_mask);
        isRobot1 = robotColor(robot1_mask);
        isRobot2 = robotColor(robot2_mask);


        cv::Mat mask;
        cv::Mat mask_inverted;
        input(boundingRect).copyTo(mask);
        cv::bitwise_not(mask, mask_inverted);
        
        if(isRobot0){
            robots_coord_[0] = robot_pose;
            cv::bitwise_and(robot0_mask, mask_inverted, roi);
            *orientation_index = 0;
            return roi;
        }
        else if(isRobot1){
            robots_coord_[1] = robot_pose;
            cv::bitwise_and(robot1_mask, mask_inverted, roi);
            *orientation_index = 1;
            return roi;
        }
        else if(isRobot2){
            robots_coord_[2] = robot_pose;
            cv::bitwise_and(robot2_mask, mask_inverted, roi);
            *orientation_index = 2;
            return roi;
        }else{
            return mask;
        }

    }
    else if (arguments_ == "theirs"){
        robots_coord_[index] = robot_pose;
        return roi;
    }
    
}

bool RobotIdentification::robotColor(cv::Mat mask){

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;

    cv::findContours(mask, contours, hierarchy, CV_RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    for(int i = 0, sum_ = 0; i < contours.size(); i++){
        auto bounding_rect = cv::boundingRect(contours[i]);
        sum_ += bounding_rect.area();
        if (sum_ > 10)
            return true;
    }

    return false;
}


void RobotIdentification::findOrientation(cv::Mat mask, int index, cv::Mat orient_circle){
    
    if (arguments_ == "ours"){
        std::vector<cv::Vec4i> hierarchy;
        std::vector<std::vector<cv::Point>> contours;
        
        cv::Moments moments;
        cv::Point2f robot_center;
        cv::Point2f robot_id;

        float area = 0;
        cv::findContours(mask, contours, hierarchy, CV_RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
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
         
        auto orientation_vector = robot_id - robot_center;
        auto theta = atan2(orientation_vector.y, orientation_vector.x);

        // cv::Point2f orient = cv::Point2f(robot_center.x + 10*cos(theta), robot_center.y + 10*sin(theta));
        // cv::circle(mask, robot_center, 2, cv::Scalar(150,0,0));
        // cv::circle(mask, robot_id, 2, cv::Scalar(150,0,0));
        // cv::line(mask, robot_center, orient, cv::Scalar(133,255,20));
        // cv::imshow("mask", mask);

        robots_orientation_[index] = theta;
    }
}
