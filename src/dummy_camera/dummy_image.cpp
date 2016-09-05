/**
 * @file   dummy_image.cpp
 * @author Gabriel Naves da Silva
 * @date   05/06/2016
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief  Dummy image node
 *
 * Loads an image and publishes it repeatedly on the raw image topic.
 */

#include <string>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>

image_transport::Publisher rgb_pub, depth_pub;
cv_bridge::CvImage rgb_frame, depth_frame;
bool using_rgb, using_depth;

void loadConfig();
void argumentCheck(int argc);
void publisherSetup(image_transport::ImageTransport it);
void loadRGBImage(int argc, char **argv);
void loadDepthImage(int argc, char **argv);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dummy_image");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh); // Used to publish and subscribe to images.

    loadConfig();
    argumentCheck(argc);
    publisherSetup(it);

    if (using_rgb)
        loadRGBImage(argc, argv);
    if (using_depth)
        loadDepthImage(argc, argv);

    // Set the loop rate
    ros::Rate loop_rate(15);

    ROS_INFO("Publishing dummy image(s)");
    while (ros::ok())
    {
        if (using_rgb) {
            rgb_pub.publish(rgb_frame.toImageMsg());
            cv::imshow("dummy rgb image", rgb_frame.image);
        }

        if (using_depth) {
            depth_pub.publish(depth_frame.toImageMsg());
            cv::imshow("dummy depth image", depth_frame.image);
        }

        cv::waitKey(1);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

void loadConfig() {
    ros::param::get("/image/using_rgb", using_rgb);
    ros::param::get("/image/using_depth", using_depth);
}

void argumentCheck(int argc) {
    int argc_size = 1;
    std::string err_msg = "Not enough arguments. Usage: dummy_image";

    if (using_rgb) {
        argc_size++;
        err_msg += " <rgb image file>";
    }

    if (using_depth) {
        argc_size++;
        err_msg += " <depth image file>";
    }

    if(argc < argc_size)
    {
        ROS_ERROR("%s", err_msg.c_str());
        exit(-1);
    }
}

void publisherSetup(image_transport::ImageTransport it) {
    if (using_rgb)
        rgb_pub = it.advertise("/camera/rgb/image_raw", 1);

    if (using_depth)
        depth_pub = it.advertise("/camera/depth/image", 1);
}

void loadRGBImage(int argc, char **argv) {
    rgb_frame.image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
    if (not rgb_frame.image.data)
    {
        ROS_ERROR("Could not find image file on %s", argv[1]);
        exit(-1);
    }

    rgb_frame.encoding = sensor_msgs::image_encodings::BGR8;

    cv::namedWindow("dummy rgb image");
}

void loadDepthImage(int argc, char **argv) {
    int depth_image_name_index = argc - 1;
    depth_frame.image = cv::imread(argv[depth_image_name_index], CV_LOAD_IMAGE_ANYDEPTH);
    if (not depth_frame.image.data)
    {
        ROS_ERROR("Could not find image file on %s", argv[depth_image_name_index]);
        exit(-1);
    }

    depth_frame.encoding = sensor_msgs::image_encodings::TYPE_32FC1;

    cv::namedWindow("dummy depth image");
}
