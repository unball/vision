#include <vector>
#include <cmath>

#include <ros/ros.h>

#include <vision/VisionMessage.h>

//const float field_x_length = 1.50;
//const float field_y_length = 1.30;

const float field_x_length = 1.60;
const float field_y_length = 1.30;

const float camera_x_length = 640;
const float camera_y_length = 480;

void receiveVisionMessage(const vision::VisionMessage::ConstPtr &msg_s);
void convertPixelsToMeters();

vision::VisionMessage message;
ros::Publisher publisher;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pixel_to_metric_conversion_node");

    ros::NodeHandle n;
    ros::Rate loop_rate(30); // Hz

    ros::Subscriber sub = n.subscribe("vision_topic", 1, receiveVisionMessage);
    publisher = n.advertise<vision::VisionMessage>("pixel_to_metric_conversion_topic", 1);

    ROS_INFO("Starting pixel-to-metric conversion node");
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
        publisher.publish(message);
    }
    
    return 0;
}

bool shouldConvert[3] = { true, true, true};
void receiveVisionMessage(const vision::VisionMessage::ConstPtr &msg_v)
{
    message = *msg_v;
    convertPixelsToMeters();
}

void convertPixelsToMeters(){
    auto x_conversion = field_x_length / camera_x_length;
    auto y_conversion = (field_y_length / camera_y_length) * -1;
    for (int i = 0; i < 6; ++i)
    {
        message.x[i] -= camera_x_length / 2;
        message.y[i] -= camera_y_length / 2;
        message.x[i] *= x_conversion;
        message.y[i] *= y_conversion;
    }
    message.ball_x -= camera_x_length / 2;
    message.ball_y -= camera_y_length / 2;
    message.ball_x *= x_conversion;
    message.ball_y *= y_conversion;
}
