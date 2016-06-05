/**
 * @file   vision_node.cpp
 * @author Matheus Vieira Portela
 * @author Gabriel Naves da Silva
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

#include <vision/VisionMessage.h>

#include <vision/vision.hpp>

void receiveRGBFrame(const sensor_msgs::ImageConstPtr& msg);
bool isValidSize(const cv::Mat &img);
void publishVisionMessage(ros::Publisher &publisher);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vision_node");
    ros::NodeHandle node_handle;
    image_transport::ImageTransport img_transport(node_handle);

    image_transport::Subscriber rgb_sub;
    rgb_sub = img_transport.subscribe("camera/rgb/image_raw", 1, receiveRGBFrame);

    ros::Publisher publisher = node_handle.advertise<vision::VisionMessage>("vision_topic", 1);

    ros::Rate loop_rate(30);
    while (ros::ok())
    {
        Vision::getInstance().run();
        publishVisionMessage(publisher);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

/**
 * Receives the RGB frame and passes it to the RawImage instance, on the vision system.
 * @param msg a ROS image message pointer.
 */
void receiveRGBFrame(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_WARN("cv_bridge exception: %s", e.what());
        return;
    }

    if (isValidSize(cv_ptr->image))
        Vision::getInstance().setRawImage(cv_ptr->image);
}

/**
 * Check whether an image has a valid size, i.e., neither the width nor the height can be zero.
 * @param img image that will be checked.
 * @return true if the image has proper size, false otherwise.
 */
bool isValidSize(const cv::Mat &img)
{
    return not (img.rows == 0 or img.cols == 0);
}

/**
 * Publishes the vision message to the vision topic.
 * @param publisher a ROS node publisher.
 */
void publishVisionMessage(ros::Publisher &publisher)
{
    vision::VisionMessage message;
    publisher.publish(message);
}
