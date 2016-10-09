#include <vision/find_robots_by_color.hpp>

REGISTER_ALGORITHM_DEF(FindRobotsColor);

void FindRobotsColor::run(){
    RawImage::getInstance().getRawRGBImage().copyTo(rgb_input_);
    preProcessor(rgb_input_);
}

void FindRobotsColor::init(){
    auto sourceDir = ros::package::getPath("vision").append("/data/");
    auto filename = "color_calibration.yaml";

    color_reader_ = cv::FileStorage(sourceDir+filename,cv::FileStorage::READ);
    if (color_reader_.isOpened())
    {
        color_reader_["Blue"] >> blue_mat_;
        color_reader_["Yellow"] >> yellow_mat_;
    }
}

void FindRobotsColor::preProcessor(cv::Mat input){
    
    cv::cvtColor(input, input, CV_BGR2HSV);
    
    std::vector<cv::Mat> channels(3);
    cv::split(input, channels);

    
    if(arguments_ == "Yellow"){
        cv::inRange(input,
                    cv::Scalar(yellow_mat_.at<int>(0,0), yellow_mat_.at<int>(0,1), yellow_mat_.at<int>(0,2)),
                    cv::Scalar(yellow_mat_.at<int>(1,0), yellow_mat_.at<int>(1,1), yellow_mat_.at<int>(1,2)),
                    mask_);
        applyMorfology();
        cv::imshow("Yellow", mask_);
    }
    else if (arguments_ == "Blue")
    {
        cv::inRange(input,
                    cv::Scalar(blue_mat_.at<int>(0,0), blue_mat_.at<int>(0,1), blue_mat_.at<int>(0,2)),
                    cv::Scalar(blue_mat_.at<int>(1,0), blue_mat_.at<int>(1,1), blue_mat_.at<int>(1,2)),
                    mask_);
        applyMorfology();
        cv::imshow("Blue", mask_);
    }
    else{
        ROS_ERROR("[FIND ROBOTS] COLOR BAD DEFINITION");
        return;
    }

    output_rgb_image_ = mask_;

    cv::waitKey(1);
}

void FindRobotsColor::applyMorfology(){
    cv::Mat element2 = getStructuringElement( cv::MORPH_RECT,
                                       cv::Size( erosion_size2 + 1, erosion_size2+1 ),
                                       cv::Point( erosion_size2, erosion_size2 ) );
    cv::erode(mask_, mask_, element2);
    // cv::dilate(mask_, mask_, element2);
    // cv::dilate(mask_, mask_, element2);

}

