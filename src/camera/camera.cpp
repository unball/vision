#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "camera");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh); // Used to publish and subscribe to images.
    image_transport::Publisher rgb_pub = it.advertise("/camera/rgb/image_raw", 1);
    cv_bridge::CvImage rgb_frame;
    ROS_INFO("Argv %s", argv[1]);
    cv::VideoCapture rgb_cap(atoi(argv[1]));

    if (!rgb_cap.isOpened())
    {
        ROS_ERROR("Could not open of find the rgb video file");
        return -1;
    }
    double rgb_video_fps = rgb_cap.get(CV_CAP_PROP_FPS);
    ros::Rate loop_rate(isnan(rgb_video_fps) ? 25 : rgb_video_fps);
    ROS_INFO("Loop rate: %lf", rgb_video_fps);

    // Set rgb and depth frame encoding
    rgb_frame.encoding = sensor_msgs::image_encodings::BGR8;
    ROS_INFO("Sending video");
    while(ros::ok()){
        rgb_cap >> rgb_frame.image;
        rgb_pub.publish(rgb_frame.toImageMsg());
        ros::spinOnce();
        loop_rate.sleep();
        // cv::imshow("RGB camera", rgb_frame.image);
        // cv::waitKey(1);
    }

    return 0;
}