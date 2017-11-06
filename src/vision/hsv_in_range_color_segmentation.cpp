#include <vision/hsv_in_range_color_segmentation.hpp>

REGISTER_ALGORITHM_DEF(HSVInRangeColorSegmentation);

static bool show_image = false;

void HSVInRangeColorSegmentation::init()
{
    if (show_image) {
        window_name_ = "HSV InRange color segmentation: " + arguments_;
        cv::namedWindow(window_name_);
    }

    auto sourceDir = ros::package::getPath("vision").append("/data/");
    auto filename = "color_calibration.yaml";
    colorReader_ = cv::FileStorage(sourceDir+filename, cv::FileStorage::READ);
    if (colorReader_.isOpened())
        colorReader_[arguments_] >> color_mat_;
}

void HSVInRangeColorSegmentation::run()
{
    cv::Mat raw_image = RawImage::getInstance().getRawImage();
    cv::cvtColor(raw_image, raw_image, CV_BGR2HSV);
    cv::Mat mask;
    cv::inRange(raw_image,
                cv::Scalar(color_mat_.at<int>(0,0), color_mat_.at<int>(0,1), color_mat_.at<int>(0,2)),
                cv::Scalar(color_mat_.at<int>(1,0), color_mat_.at<int>(1,1), color_mat_.at<int>(1,2)),
                mask);

    cv::Mat elem;
    int kernel_size = 2*kernel_size_+1;
    elem = cv::getStructuringElement(cv::MORPH_RECT,
                                     cv::Size(kernel_size, kernel_size));

    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, elem);

    if (show_image)
        cv::imshow(window_name_, mask);
    output_image_ = mask;
}
