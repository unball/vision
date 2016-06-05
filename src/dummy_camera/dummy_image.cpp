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

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dummy_image");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh); // Used to publish and subscribe to images.
    image_transport::Publisher pub = it.advertise("/camera/rgb/image_raw", 1);
    cv_bridge::CvImage frame;

    // Check if enough arguments where given
    if(argc != 2)
    {
        ROS_ERROR("Not enough arguments. Usage: dummy_image <image file>");
        return -1;
    }

    // Load the image.
    frame.image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);

    // Check if image exists
    if (not frame.image.data)
    {
        ROS_ERROR("Could not find image file on %s", argv[1]);
        return -1;
    }

    // Set the loop rate
    ros::Rate loop_rate(15);

    // Set frame encoding
    frame.encoding = sensor_msgs::image_encodings::BGR8;

    // Show the image being sent
    cv::namedWindow("dummy_image");
    cv::imshow("dummy_image", frame.image);
    cv::waitKey(1);

    // Publish the image
    ROS_INFO("Publishing dummy image");
    while (ros::ok())
    {
        pub.publish(frame.toImageMsg());
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
