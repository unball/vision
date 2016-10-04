#include "vision/robot_identification.hpp"

REGISTER_ALGORITHM_DEF(RobotIdentification);

void RobotIdentification::run(){
    depth_input_ = segmentation_algorithm_->getSegmentationDepthOutput();
    cv::Mat upperPoints = depth_input_;
    indentify(upperPoints);
}

void RobotIdentification::init(){
}

void RobotIdentification::indentify(cv::Mat input){
    cv::Mat upperPoints = input;
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
                robots_.push_back(newboundingRect);
                cv::rectangle(input, newboundingRect, cv::Scalar(133,133,133),3, 8,0); 
                cv::rectangle(rgb_input, newboundingRect, cv::Scalar(255,255,255),3, 8,0);      
                }
            }
        }
    }
    if (not hasclosed_){
        if (input.rows > 0 and input.cols > 0)
        {
            cv::imshow("white image", input);
            if (cv::waitKey(30) == 'w'){
                cv::destroyWindow("white image");
                hasclosed_ = true;  
            }
        } 
    }
}
