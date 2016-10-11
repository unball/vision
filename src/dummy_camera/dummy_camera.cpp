/**
 * @file   dummy_camera.cpp
 * @author Gabriel Naves da Silva
 * @author Matheus Vieira Portela
 * @date   21/03/2014
 *
 * @attention Copyright (C) 2014 UnBall Robot Soccer Team
 *
 * @brief  Dummy camera node
 *
 * Loads the rgb video file and publishes it on the "/camera/rgb/image_raw" topic,
 * and loads the depth images in sequence and publishes them on the
 * "/camera/depth/image_raw" topic.
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>

#include <string>
#include <cmath>

/**
 * Converts an integer to a string.
 *
 * @param num the integer to be converted.
 */
std::string to_string(int num)
{
    std::string result;
    char tmp[100];
    sprintf(tmp, "%d", num);
    result = tmp;
    return result;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dummy_camera");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh); // Used to publish and subscribe to images.
    image_transport::Publisher rgb_pub = it.advertise("/camera/rgb/image_raw", 1);
    cv_bridge::CvImage rgb_frame;
    int frame_counter; // Used to count the number of frames published on the topic.
    int num_frames;

    // Check if enough arguments where given
    if( argc != 2)
    {
        ROS_ERROR("Not enough arguments. Usage: dummy_camera <rgb video file>");
        return -1;
    }

    // Load rgb video and check for errors
    cv::VideoCapture rgb_cap(argv[1]);
    if (!rgb_cap.isOpened())
    {
        ROS_ERROR("Could not open of find the rgb video file");
        return -1;
    }

    ros::Rate loop_rate(25);
    rgb_frame.encoding = sensor_msgs::image_encodings::BGR8;
    // Publish the video
    ROS_INFO("[Dummy Camera]Sending video");
    while (ros::ok())
    {
        // Publish the rgb frame
        rgb_cap >> rgb_frame.image; // Get a new frame from the rgb video capture
        if (rgb_frame.image.empty())
            break;
        rgb_pub.publish(rgb_frame.toImageMsg());

        ros::spinOnce();
        loop_rate.sleep();
    }
    ROS_INFO("Finished sending video");

    return 0;
}
