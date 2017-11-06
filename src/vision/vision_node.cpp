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
#include <time.h>
#include <stdio.h>
#include <vision/VisionMessage.h>

#include <vision/vision.hpp>

image_transport::Subscriber image_sub;

void loadConfig();
void subscriberSetup(image_transport::ImageTransport &img_transport);
void receiveFrame(const sensor_msgs::ImageConstPtr& msg);
void publishVisionMessage(ros::Publisher &publisher);


int main(int argc, char **argv)
{
    srand(time(NULL));
    ros::init(argc, argv, "vision_node");
    ros::NodeHandle node_handle;
    image_transport::ImageTransport img_transport(node_handle);
    subscriberSetup(img_transport);

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
    publisher.publish(message);
}
