#include "calibration/color_calibration.hpp"

ColorCalibration::ColorCalibration(){
    //initial parameters
    hsv_min_h_ = 0;
    hsv_max_h_ = 0;
    hsv_min_s_ = 0;
    hsv_max_s_ = 0;
    hsv_min_v_ = 0;
    hsv_max_v_ = 0;
    is_blue_saved_ = false;
    is_yellow_saved_ = false;
    window_name_ = "Color Calibration";

    //decide if open or not file to save color calibration
    ros::param::get("/vision/calibration/calibrate_team_color", calibrate_);
    auto sourceDir = ros::package::getPath("vision").append("/data/");
    auto filename = "color_calibration.yaml";

    if (calibrate_)
    {
        //create UI
        cv::namedWindow(window_name_);
        cv::createTrackbar("HMIN", window_name_, &hsv_min_h_, 360);
        cv::createTrackbar("HMAX", window_name_, &hsv_max_h_, 360);
        cv::createTrackbar("SMIN", window_name_, &hsv_min_s_, 256);
        cv::createTrackbar("SMAX", window_name_, &hsv_max_s_, 256);
        cv::createTrackbar("VMIN", window_name_, &hsv_min_v_, 256);
        cv::createTrackbar("VMAX", window_name_, &hsv_max_v_, 256);

        //set file calibration to save HSV values
        
        colorManager_ = cv::FileStorage(sourceDir+filename, cv::FileStorage::WRITE);
    }
}

ColorCalibration::~ColorCalibration(){
    if (calibrate_)
        colorManager_.release();
}

void ColorCalibration::calibrate(cv::Mat rgb_input){
    if (calibrate_)
    {

        if (is_blue_saved_ && is_yellow_saved_)
            cv::destroyWindow(window_name_);
        else{
            cv::Mat hsv_converted;
            cv::cvtColor(rgb_input, hsv_converted, CV_BGR2HSV);

            cv::inRange(hsv_converted, cv::Scalar(hsv_min_h_, hsv_min_s_, hsv_min_v_),
                         cv::Scalar(hsv_max_h_, hsv_max_s_, hsv_max_v_), segmented_image_);

            cv::imshow(window_name_, segmented_image_);
        }

        if (cv::waitKey(10) == 'b' && not is_blue_saved_)
        {
            ROS_INFO("Blue Calibrated!");
            saveBlue();
        }
        if (cv::waitKey(10) == 'y' && not is_yellow_saved_)
        {
            ROS_INFO("Yellow Calibrated!");
            saveYellow();
        }
    }
    else
        ROS_WARN("Color already calibrated, to recalibrate change config!");
}

void ColorCalibration::saveBlue(){
    colorManager_ << "Blue";
    colorManager_ << "{" << "Min" << cv::Scalar(hsv_min_h_, hsv_min_s_, hsv_min_v_);
    colorManager_        << "Max" << cv::Scalar(hsv_max_h_, hsv_max_s_, hsv_max_v_);

    colorManager_ << "}";
    is_blue_saved_ = true;
}

void ColorCalibration::saveYellow(){
    colorManager_ << "Yellow";
    colorManager_ << "{" << "Min" << cv::Scalar(hsv_min_h_, hsv_min_s_, hsv_min_v_);
    colorManager_        << "Max" << cv::Scalar(hsv_max_h_, hsv_max_s_, hsv_max_v_);

    colorManager_ << "}";
    is_yellow_saved_ = true;
}

bool ColorCalibration::isCalibrated(){
    return calibrate_;
}