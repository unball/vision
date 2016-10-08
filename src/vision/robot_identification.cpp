#include "vision/robot_identification.hpp"

REGISTER_ALGORITHM_DEF(RobotIdentification);

void RobotIdentification::run(){
    rgb_input_ = segmentation_algorithm_->getSegmentationRGBOutput();
    RawImage::getInstance().getRawRGBImage().copyTo(rgb_input_);
    cv::Mat rgb_input = rgb_input_;
    
    find(rgb_input_);

    identify(rgb_input);
}

void RobotIdentification::init(){
    auto sourceDir = ros::package::getPath("vision").append("/data/");
    auto filename = "color_calibration.yaml";

    colorReader_ = cv::FileStorage(sourceDir+filename,cv::FileStorage::READ);
    if (colorReader_.isOpened())
    {
        colorReader_["Blue"] >> blue_mat_;
        colorReader_["Yellow"] >> yellow_mat_;
    }

}

void RobotIdentification::find_depth(cv::Mat input){
    cv::Mat depth_input = input;
    cv::Mat rgb_input = rgb_input_;

    std::vector<std::vector<cv::Point>> newcontours;
    std::vector<cv::Vec4i> newhierarchy;

    uchar *p;
    cv::dilate(depth_input, depth_input, cv::Mat());

    cv::createTrackbar("Area", "white image", &area_, 2000);

    for (int i = 0; i < depth_input.rows; ++i)
    {
        p = depth_input.ptr<uchar>(i);
        for (int j = 0; j < depth_input.cols; ++j)
        {
            if ((int)p[j] == 255)
            {
                int area = 10;
                bool islost = true;
                
                for (int k = 0; k < 20 and j+k < depth_input.cols; ++k)
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

    cv::findContours(depth_input, newcontours, newhierarchy, CV_RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    auto color = cv::Scalar::all(255);
    for (uint i = 0; i < newcontours.size(); ++i)
    {
        auto newboundingRect = cv::boundingRect(newcontours[i]);

        if (newboundingRect.area() > area_)
        {
            cv::drawContours(depth_input, newcontours, i, color, 3, 8, newhierarchy);
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

void RobotIdentification::identify(cv::Mat rgb_input){
    auto robots = robots_;
    std::vector<char> teams;


    for (int i = 0; i < robots.size(); ++i)
    {
        cv::Mat robotROI = rgb_input(robots[i]);
        cv::imshow("window", robotROI);
        cv::waitKey(1);

    }

}