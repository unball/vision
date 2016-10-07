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
    is_orange_saved_ = false;
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

        //save previous parameters
        colorHandler_ = cv::FileStorage(sourceDir+filename, cv::FileStorage::READ);
        old_blue = colorHandler_["Blue"];

        old_yellow = colorHandler_["Yellow"];
        old_orange = colorHandler_["Orange"];
        
        //set file calibration to save HSV values
        colorManager_ = cv::FileStorage(sourceDir+filename, cv::FileStorage::WRITE);
    }
}

ColorCalibration::~ColorCalibration(){
    if (calibrate_)
        if (not is_blue_saved_)
        {
            cv::FileNodeIterator it = old_blue.begin();
            cv::Scalar min = (cv::Scalar)(*it)["Min"];
            cv::Scalar max = (cv::Scalar)(*it)["Max"];
            colorManager_ << "Blue";
            colorManager_ << "{" << "Min" << min;
            colorManager_        << "Max" << max;
            colorManager_ << "}";
        }
        if (not is_yellow_saved_)
        {
            
            cv::FileNodeIterator it = old_yellow.begin();
            cv::Scalar min = (cv::Scalar)(*it)["Min"];
            cv::Scalar max = (cv::Scalar)(*it)["Max"];
            colorManager_ << "Yellow";
            colorManager_ << "{" << "Min" << min;
            colorManager_        << "Max" << max;
            colorManager_ << "}";
        }
        if(not is_orange_saved_){
            cv::FileNodeIterator it = old_orange.begin();
            cv::Scalar min = (cv::Scalar)(*it)["Min"];
            cv::Scalar max = (cv::Scalar)(*it)["Max"];
            colorManager_ << "Orange";
            colorManager_ << "{" << "Min" << min;
            colorManager_        << "Max" << max;
            colorManager_ << "}";
        }
        colorHandler_.release();
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
            save("Blue");
        }
        if (cv::waitKey(10) == 'y' && not is_yellow_saved_)
        {
            ROS_INFO("Yellow Calibrated!");
            save("Yellow");
        }
        if (cv::waitKey(10) == 'o' && not is_orange_saved_)
        {
            ROS_INFO("Orange Calibrated!");
            save("Orange");
        }
    }
    else
        ROS_WARN("Color already calibrated, to recalibrate change config!");
}

void ColorCalibration::save(std::string color){
    colorManager_ << color;
    colorManager_ << "{" << "Min" << cv::Scalar(hsv_min_h_, hsv_min_s_, hsv_min_v_);
    colorManager_        << "Max" << cv::Scalar(hsv_max_h_, hsv_max_s_, hsv_max_v_);
    colorManager_ << "}";
    if (color == "Blue")
        is_blue_saved_ = true;
    if (color == "Yellow")
        is_yellow_saved_ = true;
    if (color == "Orange")
        is_orange_saved_ = true;
}

bool ColorCalibration::isCalibrated(){
    return calibrate_;
}