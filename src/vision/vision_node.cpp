/**
 * @file   vision_node.cpp
 * @author Matheus Vieira Portela
 * @author Gabriel Naves da Silva
 * @author Manoel Vieira Coelho Neto
 * @date   25/03/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief Computer vision node
 *
 * This node subscribes to the camera or dummy_camera topic, applies
 * computer vision algorithms to extract robots positions and publishes
 * to the vision topic.
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <time.h>
#include <stdio.h>
#include <vision/VisionMessage.h>
#include <vision/vision.hpp>

#include <calibration/matching.hpp>
#include <calibration/select_field.hpp>
#include <calibration/color_calibration.hpp>

image_transport::Subscriber image_sub;

cv::VideoCapture rgb_cap;

bool selecterstarted = false;

void loadConfig();
void subscriberSetup(image_transport::ImageTransport &img_transport);
void receiveFrame(const sensor_msgs::ImageConstPtr& msg);
void publishVisionMessage(ros::Publisher &publisher);
cv::Mat getFrameFromCamera();
cv::Mat calibrate(cv::Mat frame, SelectField *selecter);


int main(int argc, char **argv)
{
    std::string rgb_select_name = "RGB Calibration";
    
    srand(time(NULL));
    ros::init(argc, argv, "vision_node");
    ros::NodeHandle node_handle;
    image_transport::ImageTransport img_transport(node_handle);
    // subscriberSetup(img_transport);

    ros::Publisher publisher = node_handle.advertise<vision::VisionMessage>("vision_topic", 1);
    ros::Rate loop_rate(30);

    rgb_cap = cv::VideoCapture(std::atoi(argv[1]));

    bool show_image=true;
    // ros::param::get("/vision/camera/show_image", show_image);
    cv::Mat camera_frame;

    
    
    if (!rgb_cap.isOpened())
    {
        ROS_ERROR("Could not open of find the rgb video file");
        return -1;
    }

    //instatiating things for calibration
    SelectField selecter(rgb_select_name, true);

    ros::param::get("/vision/calibration/calibrate_rectify_matrix", selecterstarted);
    selecterstarted = not selecterstarted;

    cv::Mat rgb_fixed;
    bool showframes = true;    
    

    while (ros::ok())
    {
        camera_frame = getFrameFromCamera();
        camera_frame = calibrate(camera_frame, &selecter);
        Vision::getInstance().setRawImage(camera_frame);
        Vision::getInstance().run();
        publishVisionMessage(publisher);
        ros::spinOnce();
        if (show_image)
        {
            cv::imshow("Camera Raw", camera_frame);
            cv::waitKey(1);
        }
        loop_rate.sleep();
    }

    return 0;
}

void subscriberSetup(image_transport::ImageTransport &img_transport) {
      image_sub = img_transport.subscribe("/camera/rgb/image_calibrated", 1, receiveFrame);
}

/**
 * Receives the RGB frame and passes it to the RawImage instance, on the vision system.
 * @param msg a ROS image message pointer.
 */
void receiveFrame(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    Vision::getInstance().setRawImage(cv_ptr->image);
}

/**
 * Publishes the vision message to the vision topic.
 * @param publisher a ROS node publisher.
 */
void publishVisionMessage(ros::Publisher &publisher)
{
    vision::VisionMessage message;
    auto vision_output = Vision::getInstance().getVisionOutput();

    if (vision_output.find("ball") != vision_output.end())
    {
        if (vision_output["ball"].positions.size() > 0)
        {
            message.ball_x = vision_output["ball"].positions[0].x;
            message.ball_y = vision_output["ball"].positions[0].y;
        }
        else{
            message.ball_x = 0;
            message.ball_y = 0;
        }
    }
    std::string prefix = "robot_";
    for (int i = 0; i < 3; ++i)
    {
        std::string robot = prefix + std::to_string(i);
        if (vision_output.find(robot) != vision_output.end())
        {
            if (vision_output[robot].positions.size() == 1)
            {
                message.x[i] = vision_output[robot].positions[0].x;
                message.y[i] = vision_output[robot].positions[0].y;
                message.th[i] = vision_output[robot].orientations[0];
                message.found[i] = vision_output[robot].found[0];
            }
        }
    }

    for (int i = 3; i < 6; ++i)
    {
        message.found[i] = true;
    }
    publisher.publish(message);
}

cv::Mat getFrameFromCamera(){
    cv::Mat frame;
    rgb_cap >> frame;
    return frame;
}

cv::Mat calibrate(cv::Mat frame, SelectField *selecter){
    cv::Mat rgb_fixed = frame;

    if (not selecterstarted){
        selecter->start();
        selecterstarted = true;
    }
    else if (not selecter->isDone()){
        if (not selecter->isDone())
        {
            selecter->showFrame(frame);
            selecter->run();
        }
    }
    else{
        /*At this point all clicks must be done*/
        rgb_fixed = selecter->warp(frame);
    }
    return rgb_fixed;
}