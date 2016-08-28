#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>

static cv_bridge::CvImage frame;

void receiveRGBFrame(const sensor_msgs::ImageConstPtr& msg);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "calibration_node");
    ros::NodeHandle node_handle;
    image_transport::ImageTransport it(node_handle);

    image_transport::Subscriber rgb_sub;
    rgb_sub = it.subscribe("/camera/rgb/image_raw", 1, receiveRGBFrame);

    image_transport::Publisher pub = it.advertise("/camera/rgb/image_calibrated", 1);

    // Set loop rate
    ros::Rate loop_rate(15);

    // Set frame encoding
    frame.encoding = sensor_msgs::image_encodings::BGR8;

    cv::namedWindow("Calibrated Image");
    while (ros::ok())
    {
        pub.publish(frame.toImageMsg());
        ros::spinOnce();
        loop_rate.sleep();
        if (not (frame.image.rows == 0 or frame.image.cols == 0)) {
            cv::imshow("Calibrated Image", frame.image);
            cv::waitKey(1);
        }
    }

    return 0;
}

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

    frame.image = cv_ptr->image;
}
