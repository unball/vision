#include <vector>
#include <cmath>

#include <ros/ros.h>

#include <vision/VisionMessage.h>
#include <vision/PixelToMetricConversionMessage.h>

//const float field_x_length = 1.50;
//const float field_y_length = 1.30;

const float field_x_length = 1.60;
const float field_y_length = 1.30;

const float camera_x_length = 640;
const float camera_y_length = 480;

void receiveVisionMessage(const vision::VisionMessage::ConstPtr &msg_s);
void convertPixelsToMeters();

vision::PixelToMetricConversionMessage message;
ros::Publisher publisher;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pixel_to_metric_conversion_node");

    ros::NodeHandle n;
    ros::Rate loop_rate(10); // Hz

    ros::Subscriber sub = n.subscribe("vision_topic", 1, receiveVisionMessage);
    publisher = n.advertise<vision::PixelToMetricConversionMessage>("pixel_to_metric_conversion_topic", 1);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

bool shouldConvert[3] = { true, true, true};
void receiveVisionMessage(const vision::VisionMessage::ConstPtr &msg_v)
{
    std::vector<float> x(6), y(6), th(6);
    std::vector<float> ball_location(2);

    //ROS_INFO("\n\n[PxToMConvNode]:ReceiveVisionMessage - Receiving vision message");

    for (int robot_index = 0; robot_index < 6; robot_index++)
    {
        //ROS_INFO("%d x: %f\t y: %f\t th: %f", robot_index, msg_v->x[robot_index], msg_v->y[robot_index],msg_v->th[robot_index]);

        if(msg_v->y[robot_index] > 1 and msg_v->y[robot_index] < 480)
        {
            message.x[robot_index] = msg_v->x[robot_index];
            message.y[robot_index] = msg_v->y[robot_index];
            if (not std::isnan(msg_v->th[robot_index])){
                message.th[robot_index] = msg_v->th[robot_index];
            }
            shouldConvert[robot_index] = true;
        }
        else
            shouldConvert[robot_index] = false;
    }
    message.ball_x = msg_v->ball_x;
    message.ball_y = msg_v->ball_y;
    convertPixelsToMeters();

    //ROS_INFO("\n\n[MeasurementNode]:ReceiveVisionMessage - Sending measurement system message");

    for (int robot_index = 0; robot_index < 6; robot_index++)
    {
        //ROS_INFO("%d x: %f\t y: %f\t th: %f", robot_index, message.x[robot_index], message.y[robot_index],message.th[robot_index]);
    }
    //ROS_INFO("Ball: x: %f, y: %f", message.ball_x, message.ball_y);

    publisher.publish(message);
}

void convertPixelsToMeters(){
    auto x_conversion = field_x_length / camera_x_length;
    auto y_conversion = (field_y_length / camera_y_length) * -1;
    for (int i = 0; i < 6; ++i)
    {
        if (i<3)
        {
            if (shouldConvert[i])
            {
                message.x[i] -= camera_x_length / 2;
                message.y[i] -= camera_y_length / 2;
                message.x[i] *= x_conversion;
                message.y[i] *= y_conversion;
            }
        }
        else
        {
            message.x[i] -= camera_x_length / 2;
            message.y[i] -= camera_y_length / 2;
            message.x[i] *= x_conversion;
            message.y[i] *= y_conversion;
        }
    }
    message.ball_x -= camera_x_length / 2;
    message.ball_y -= camera_y_length / 2;
    message.ball_x *= x_conversion;
    message.ball_y *= y_conversion;
}
