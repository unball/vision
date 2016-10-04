#include <vision/findrobots.hpp>

REGISTER_ALGORITHM_DEF(FindRobots);

void FindRobots::run(){
    RawImage::getInstance().getRawDepthImage().copyTo(input_);
    RawImage::getInstance().getRawRGBImage().copyTo(rgb_input_);
    cv::Mat upperPoints = preProcessor(input_);
    find(upperPoints);
    output_depth_image_ = depth_to_pub_;
    output_rgb_image_ = rgb_input_;
}

void FindRobots::init(){
    //cv::namedWindow(window_name_);
    cv::createTrackbar("Canny Thresh 1", window_name_, &cannythresh1_, 100);
    cv::createTrackbar("Canny Thresh 2", window_name_, &cannythresh2_, 300);

}

cv::Mat FindRobots::preProcessor(cv::Mat input){
    cv::Mat cannyMat;
    std::vector<cv::Mat> channels(3);

    input.copyTo(cannyMat);
    
    cv::split(cannyMat, channels);
    cv::Canny(channels[2], cannyMat, cannythresh1_, cannythresh2_, 3);
    // cv::imshow(window_name_, cannyMat);
    if(cv::waitKey(30) == 'r')
        cv::destroyWindow(window_name_);
    
    return cannyMat;
}

void FindRobots::find(cv::Mat input){
    cv::Mat upperPoints =  input;
    extractPoints(upperPoints);

}

void FindRobots::extractPoints(cv::Mat& upperPoints){
    auto color = cv::Scalar::all(0);
    cv::drawContours(upperPoints, contours_, -1, color, 6, 10, hierarchy_);
    int const max_kernel_size = 21;

    cv::createTrackbar( "Kernel size:\n 2n +1", "RGB Video", &erosion_size, max_kernel_size);
    cv::Mat element = cv::getStructuringElement( cv::MORPH_RECT,
                                       cv::Size( 4*erosion_size + 1, 4*erosion_size + 1 ),
                                       cv::Point( erosion_size, erosion_size ));
    cv::createTrackbar( "Kernel size 2:\n 2n +1", "RGB Video", &erosion_size2, max_kernel_size);

    cv::Mat element2 = getStructuringElement( cv::MORPH_RECT,
                                       cv::Size( 4*erosion_size2 + 1, 4*erosion_size2+1 ),
                                       cv::Point( erosion_size2, erosion_size2 ) );
    cv::dilate(upperPoints, upperPoints, element);
    cv::dilate(upperPoints, upperPoints, element);
    cv::erode(upperPoints, upperPoints, element2);

    depth_to_pub_ = upperPoints;
}


