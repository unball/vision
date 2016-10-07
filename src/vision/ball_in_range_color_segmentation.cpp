#include <vision/ball_in_range_color_segmentation.hpp>

REGISTER_ALGORITHM_DEF(BallInRangeColorSegmentation);

void BallInRangeColorSegmentation::init()
{
    // ROS_INFO("Initializing the ball segmentation algorithm using inrange method.");
    cv::namedWindow(window_name_);

    auto sourceDir = ros::package::getPath("vision").append("/data/");
    auto filename = "color_calibration.yaml";
    colorReader_ = cv::FileStorage(sourceDir+filename, cv::FileStorage::READ);
    if (colorReader_.isOpened())
    {
        cv::Mat orangeMat;
        colorReader_["Orange"] >> orangeMat;
        
        //(*it)["Min"] >> testvector;
    }

    //hsv_max_ = (cv::Scalar)(*it)["Max"];
    ROS_ERROR("VIU?");    
}

void BallInRangeColorSegmentation::run()
{
    // ROS_INFO("Running the ball segmentation algorithm using inrange method.");
    // RawImage::getInstance().getRawRGBImage().copyTo(output_rgb_image_);
    /*auto rgb_image = RawImage::getInstance().getRawRGBImage();
    cv::cvtColor(rgb_image, rgb_image, CV_BGR2HSV);

    cv::Mat mask;
    cv::inRange(rgb_image,
                cv::Scalar(hsv_min_[0], hsv_min_[1], hsv_min_[2]),
                cv::Scalar(hsv_max_[0], hsv_max_[1], hsv_max_[2]),
                mask);

    cv::Mat elem;
    int kernel_size = 2*kernel_size_+1;
    elem = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                     cv::Size(kernel_size, kernel_size));

    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, elem);

    cv::imshow(window_name_, mask);
    output_rgb_image_ = mask;
    RawImage::getInstance().getRawDepthImage().copyTo(output_depth_image_);*/
}
