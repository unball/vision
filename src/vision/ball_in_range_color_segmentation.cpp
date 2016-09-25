#include <vision/ball_in_range_color_segmentation.hpp>

REGISTER_ALGORITHM_DEF(BallInRangeColorSegmentation);

void BallInRangeColorSegmentation::init()
{
    ROS_INFO("Initializing the ball segmentation algorithm using inrange method.");
    cv::namedWindow(window_name_);
    // cv::createTrackbar("kernel size", window_name_, &kernel_size_, 10);
    // cv::createTrackbar("hsv_max_v_", window_name_, &hsv_max_v_, 255);
}

void BallInRangeColorSegmentation::run()
{
    ROS_INFO("Running the ball segmentation algorithm using inrange method.");
    // RawImage::getInstance().getRawRGBImage().copyTo(output_rgb_image_);
    auto rgb_image = RawImage::getInstance().getRawRGBImage();
    cv::cvtColor(rgb_image, rgb_image, CV_BGR2HSV);

    cv::Mat mask;
    cv::inRange(rgb_image,
                cv::Scalar(hsv_min_h_, hsv_min_s_, hsv_min_v_),
                cv::Scalar(hsv_max_h_, hsv_max_s_, hsv_max_v_),
                mask);

    cv::Mat elem;
    int kernel_size = 2*kernel_size_+1;
    elem = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                     cv::Size(kernel_size, kernel_size));

    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, elem);

    cv::imshow(window_name_, mask);
    output_rgb_image_ = mask;
    RawImage::getInstance().getRawDepthImage().copyTo(output_depth_image_);
}
