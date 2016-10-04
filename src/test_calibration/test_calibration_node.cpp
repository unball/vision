#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

image_transport::Subscriber rgb_sub, depth_sub;
image_transport::Publisher rgb_pub, depth_pub;
cv_bridge::CvImage rgb_frame, depth_frame;

void rgbSetup(image_transport::ImageTransport &it);
void depthSetup(image_transport::ImageTransport &it);
void receiveRGBFrame(const sensor_msgs::ImageConstPtr& msg);
void receiveDepthFrame(const sensor_msgs::ImageConstPtr& msg);
void showFrames(cv::Mat rgb_frame_, cv::Mat depth_frame_);
bool isImageValid(cv::Mat image);
void CallBackFunc(int event, int x, int y, int flags, void* userdata);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_calibration_node");
    ros::NodeHandle node_handle;
    image_transport::ImageTransport it(node_handle);

    rgbSetup(it);
    depthSetup(it);

    // Set loop rate
    ros::Rate loop_rate(30);

    cv::namedWindow("Calibrated RGB frame");
    cv::namedWindow("Calibrated Depth frame", 1);
    cv::setMouseCallback("Calibrated Depth frame", CallBackFunc, NULL);


    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();

        showFrames(rgb_frame.image, depth_frame.image);
    }

    return 0;
}

void rgbSetup(image_transport::ImageTransport &it) {
    rgb_sub = it.subscribe("/camera/rgb/image", 1, receiveRGBFrame);
    rgb_frame.encoding = sensor_msgs::image_encodings::BGR8;
}

void depthSetup(image_transport::ImageTransport &it) {
    depth_sub = it.subscribe("/camera/depth/image", 1, receiveDepthFrame);
    depth_frame.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
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

void showFrames(cv::Mat rgb_frame_, cv::Mat depth_frame_) {
    if (not (rgb_frame_.rows == 0 or rgb_frame_.cols == 0))
        cv::imshow("Calibrated RGB frame", rgb_frame_);

    if (not (depth_frame.image.rows == 0 or depth_frame.image.cols == 0))
        cv::imshow("Calibrated Depth frame", depth_frame_);

    if(cv::waitKey(30) == 27)
        cv::destroyAllWindows();
}

bool isImageValid(cv::Mat image) {
    return (image.rows > 0 and image.cols > 0);
}


void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
     if  ( event == cv::EVENT_LBUTTONDOWN )
     {
        std::cout << "hey\n";
        auto color = cv::Scalar(-1);
        cv::circle(rgb_frame.image, cv::Point(x,y), 3, color);
        cv::circle(depth_frame.image, cv::Point(x,y), 3, color);
     }
}