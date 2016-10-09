#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <calibration/matching.hpp>
#include <calibration/select_field.hpp>
#include <calibration/depth_fix.hpp>
#include <calibration/color_calibration.hpp>

image_transport::Subscriber rgb_sub, depth_sub;
image_transport::Publisher rgb_pub, depth_pub;
cv_bridge::CvImage rgb_frame, depth_frame;
cv_bridge::CvImage rgb_frame_to_pub, depth_frame_to_pub;
bool using_rgb, using_depth;

void loadConfig();
void rgbSetup(image_transport::ImageTransport &it);
void depthSetup(image_transport::ImageTransport &it);
void receiveRGBFrame(const sensor_msgs::ImageConstPtr& msg);
void receiveDepthFrame(const sensor_msgs::ImageConstPtr& msg);
void publishFrames();
void showFrames(cv::Mat rgb_frame_);
bool isImageValid(cv::Mat image);

int main(int argc, char **argv)
{
    std::string rgb_match_name = "RGB Calibration matching";
    std::string depth_match_name = "Depth Calibration matching";
    std::string rgb_select_name = "RGB Calibration";
    std::string depth_select_name = "Depth Calibration";

    ros::init(argc, argv, "calibration_node");
    ros::NodeHandle node_handle;
    image_transport::ImageTransport it(node_handle);

    loadConfig();
    rgbSetup(it);
    depthSetup(it);

    // Set loop rate
    ros::Rate loop_rate(30);

    SelectField selecter(rgb_select_name, true);
    SelectField selecter_depth(depth_select_name, false);
    ColorCalibration color_calib;
    DepthFix depth_fixer;

    bool selecterstarted = false;
    ros::param::get("/vision/calibration/calibrate_rectify_matrix", selecterstarted);
    selecterstarted = not selecterstarted;

    cv::Mat rgb_fixed;
    cv::Mat depth_fixed;
    bool showframes = true;

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();

        if (not isImageValid(rgb_frame.image))
            continue;

        if (not selecterstarted){
            selecter.start();
            selecter_depth.start();
            selecterstarted = true;
        }
        else if (not selecter.isDone() || not selecter_depth.isDone()){
            if (not selecter.isDone())
            {
                selecter.showFrame(rgb_frame.image);
                selecter.run();
            }
        }
        else{
            /*At this point all clicks must be done*/

            rgb_fixed = selecter.warp(rgb_frame.image);


            color_calib.calibrate(rgb_fixed);
            rgb_frame_to_pub.image = rgb_fixed;
            
            if (showframes)
                showFrames(rgb_fixed);

            if (cv::waitKey(30) == 'c')
            {
                cv::destroyWindow("Calibrated RGB frame");
                showframes = false;                
            }

            if (showframes)
                showFrames(rgb_fixed);
            publishFrames();
        }
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
        rgb_frame_to_pub.encoding = sensor_msgs::image_encodings::BGR8;
    }
}

void depthSetup(image_transport::ImageTransport &it) {
    if (using_depth) {
        depth_sub = it.subscribe("/camera/depth/image", 1, receiveDepthFrame);
        depth_pub = it.advertise("/camera/depth/image_calibrated", 1);
        depth_frame.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
        
        depth_frame_to_pub.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
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
    rgb_frame_to_pub.image = cv_ptr->image;
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
        rgb_pub.publish(rgb_frame_to_pub.toImageMsg());

    if (using_depth)
        depth_pub.publish(depth_frame_to_pub.toImageMsg());
}

void showFrames(cv::Mat rgb_frame_) {
    if (using_rgb and not (rgb_frame_.rows == 0 or rgb_frame_.cols == 0))
        cv::imshow("Calibrated RGB frame", rgb_frame_);

    //if (using_depth and not (depth_frame.image.rows == 0 or depth_frame.image.cols == 0))
    //    cv::imshow("Calibrated Depth frame", depth_frame_);
}

bool isImageValid(cv::Mat image) {
    return (image.rows > 0 and image.cols > 0);
}
