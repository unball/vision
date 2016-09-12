#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "calibration/matching.hpp"
#include "calibration/select_field.hpp"

image_transport::Subscriber rgb_sub, depth_sub;
image_transport::Publisher rgb_pub, depth_pub;
cv_bridge::CvImage rgb_frame, depth_frame;
bool using_rgb, using_depth;

void loadConfig();
void rgbSetup(image_transport::ImageTransport &it);
void depthSetup(image_transport::ImageTransport &it);
void receiveRGBFrame(const sensor_msgs::ImageConstPtr& msg);
void receiveDepthFrame(const sensor_msgs::ImageConstPtr& msg);
void publishFrames();
void showFrames();
void createWindows(std::string rgb_name, std::string depth_name);

int main(int argc, char **argv)
{
    std::string rgb_match_name = "RGB Calibration matching";
    std::string depth_match_name = "Depth Calibration matching";
    std::string rgb_select_name = "RGB Calibration";

    ros::init(argc, argv, "calibration_node");
    ros::NodeHandle node_handle;
    image_transport::ImageTransport it(node_handle);

    loadConfig();
    rgbSetup(it);
    depthSetup(it);

    // Set loop rate
    ros::Rate loop_rate(45);

    createWindows(rgb_match_name, depth_match_name);
    Matching matcher(rgb_match_name, depth_match_name);
    SelectField selecter(rgb_select_name);

    bool selecterstarted = false;
    while (ros::ok())
    {
        if(not matcher.isDone()){
            matcher.showFrames(rgb_frame.image, depth_frame.image);
            matcher.run();
        }
        else if (not selecterstarted){
            selecter.start();
            selecterstarted = true;
        }
        else if (not selecter.isDone()){
            selecter.showFrame(rgb_frame.image);
            selecter.run();
        }

        publishFrames();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

void loadConfig() {
    ros::param::get("/image/using_rgb", using_rgb);
    ros::param::get("/image/using_depth", using_depth);
}

void rgbSetup(image_transport::ImageTransport &it) {
    if (using_rgb) {
        rgb_sub = it.subscribe("/camera/rgb/image_raw", 1, receiveRGBFrame);
        rgb_pub = it.advertise("/camera/rgb/image_calibrated", 1);
        rgb_frame.encoding = sensor_msgs::image_encodings::BGR8;
    }
}

void depthSetup(image_transport::ImageTransport &it) {
    if (using_depth) {
        depth_sub = it.subscribe("/camera/depth/image", 1, receiveDepthFrame);
        depth_pub = it.advertise("/camera/depth/image_calibrated", 1);
        depth_frame.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    }
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
        ROS_ERROR("cv_bridge rgb exception: %s", e.what());
        return;
    }

    rgb_frame.image = cv_ptr->image;
}

void receiveDepthFrame(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge depth exception: %s", e.what());
        return;
    }

    depth_frame.image = cv_ptr->image;
}

void publishFrames() {
    if (using_rgb)
        rgb_pub.publish(rgb_frame.toImageMsg());

    if (using_depth)
        depth_pub.publish(depth_frame.toImageMsg());
}

void showFrames() {
    if (using_rgb and not (rgb_frame.image.rows == 0 or rgb_frame.image.cols == 0))
        cv::imshow("Calibrated RGB frame", rgb_frame.image);

    if (using_depth and not (depth_frame.image.rows == 0 or depth_frame.image.cols == 0))
        cv::imshow("Calibrated Depth frame", depth_frame.image);

    cv::waitKey(1);
}

void createWindows(std::string rgb_name, std::string depth_name){
    cv::namedWindow(rgb_name);
    cv::namedWindow(depth_name);

}