#include <vision/findrobots.hpp>

REGISTER_ALGORITHM_DEF(FindRobots);

void FindRobots::run(){
    RawImage::getInstance().getRawDepthImage().copyTo(input_);
    RawImage::getInstance().getRawRGBImage().copyTo(rgb_input_);
    cv::Mat upperPoints = preProcessor(input_);
    find(upperPoints);
}

void FindRobots::init(){
    cv::namedWindow(window_name_);
    cv::createTrackbar("Canny Thresh 1", window_name_, &cannythresh1_, 100);
    cv::createTrackbar("Canny Thresh 2", window_name_, &cannythresh2_, 300);

}

cv::Mat FindRobots::preProcessor(cv::Mat input){
    cv::Mat cannyMat;
    std::vector<cv::Mat> channels(3);

    input.copyTo(cannyMat);
    
    cv::split(cannyMat, channels);
    cv::Canny(channels[2], cannyMat, cannythresh1_, cannythresh2_, 3);
    cv::imshow(window_name_, cannyMat);
    if(cv::waitKey(30) == 'r')
        cv::destroyWindow(window_name_);
    
    return cannyMat;
}

void FindRobots::find(cv::Mat input){
    cv::Mat upperPoints =  input;
    extractPoints(upperPoints);
    cv::Mat rgb_input = rgb_input_;

    std::vector<std::vector<cv::Point>> newcontours;
    std::vector<cv::Vec4i> newhierarchy;

    uchar *p;
    cv::dilate(upperPoints, upperPoints, cv::Mat());

    cv::createTrackbar("Area", "white image", &area_, 2000);

    for (int i = 0; i < upperPoints.rows; ++i)
    {
        p = upperPoints.ptr<uchar>(i);
        for (int j = 0; j < upperPoints.cols; ++j)
        {
            if ((int)p[j] == 255)
            {
                int area = 10;
                bool islost = true;
                
                for (int k = 0; k < 20 and j+k < upperPoints.cols; ++k)
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

    cv::findContours(upperPoints, newcontours, newhierarchy, CV_RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    auto color = cv::Scalar::all(255);
    for (uint i = 0; i < newcontours.size(); ++i)
    {
        auto newboundingRect = cv::boundingRect(newcontours[i]);

        if (newboundingRect.area() > area_)
        {
            cv::drawContours(upperPoints, newcontours, i, color, 3, 8, newhierarchy);
            float height = float(newboundingRect.height);
            float width = float(newboundingRect.width);
            int x = (newboundingRect.width/2) + newboundingRect.x;
            int y = (newboundingRect.height/2) + newboundingRect.y;
            float ratio = height/width;
            if ((ratio) < 1.3  and (ratio) > 0.73)
            {   
                if (x > 64 and x < 576 and y > 48 and y < 432)
                {
                robots.push_back(newboundingRect);
                cv::rectangle(input, newboundingRect, cv::Scalar(133,133,133),3, 8,0); 
                cv::rectangle(rgb_input, newboundingRect, cv::Scalar(255,255,255),3, 8,0);      
                }
            }
        }
    }

    if (not hasclosed_){
        if (upperPoints.rows > 0 and upperPoints.cols > 0)
        {
            cv::imshow("white image", input);
            cv::imshow("white rgb image", rgb_input);
            if (cv::waitKey(30) == 'w'){
                cv::destroyWindow("white image");
                cv::destroyWindow("white rgb image");
                hasclosed_ = true;  
            }
        } 
    }
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
   
}

std::vector<cv::Rect> FindRobots::getRobots(){
    auto toreturn = robots;
    robots.clear();
    return toreturn;
}

