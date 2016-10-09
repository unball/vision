#include "vision/robot_identification.hpp"

REGISTER_ALGORITHM_DEF(RobotIdentification);

void RobotIdentification::run(){
    mask_ = segmentation_algorithm_->getSegmentationRGBOutput();
    cv::Mat mask = mask_;
    
    find(mask);
    identify();
    findOrientation(mask);
}

void RobotIdentification::init(){
    window_name_ = segmentation_algorithm_->getFullName();
    cv::namedWindow(window_name_);
    cv::createTrackbar("Area", window_name_, &area_, 2000);
}

void RobotIdentification::find(cv::Mat input){
    cv::Mat mask = input;

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

    auto color = cv::Scalar::all(255);
    for (uint i = 0; i < newcontours.size(); ++i)
    {
        auto newboundingRect = cv::boundingRect(newcontours[i]);

        if (newboundingRect.area() > area_)
        {
            cv::drawContours(mask, newcontours, i, color, 3, 8, newhierarchy);
            float height = float(newboundingRect.height);
            float width = float(newboundingRect.width);
            int x = (newboundingRect.width/2) + newboundingRect.x;
            int y = (newboundingRect.height/2) + newboundingRect.y;
            float ratio = height/width;
            robots_.push_back(newboundingRect);
            cv::rectangle(mask, newboundingRect, cv::Scalar(133,133,133),3, 8,0); 
        }
    }
    if (not hasclosed_){
        if (mask.rows > 0 and mask.cols > 0)
        {
            cv::imshow(window_name_, mask);
            if (cv::waitKey(30) == 'w'){
                cv::destroyWindow(window_name_);
                hasclosed_ = true;  
            }
        } 
    }
}

void RobotIdentification::identify(){
    auto robots = robots_; 

    for (int i = 0; i < robots.size(); ++i)
    {
        int x = (robots[i].width/2) + robots[i].x; 
        int y = (robots[i].height/2) + robots[i].y;
        coord_.push_back(cv::Point(x,y));
    }

}

void RobotIdentification::findOrientation(cv::Mat input){
    auto robots = robots_;
    cv::Mat mask = input;
    std::vector<cv::Rect> little_ball;
    std::vector<cv::Point> orientation;

}