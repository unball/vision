#!/usr/bin/env python

import rospy

from vision.msg import VisionMessage

def main():
    rospy.init_node('kalman_topic', anonymous=True)

    while not rospy.is_shutdown():
        rospy.Subscriber('pixel_to_metric_conversion_topic', VisionMessage, subscriber)
        rospy.spin()
    pass


def subscriber(data):
    
    rate = rospy.Rate(10)
    
    x_curr = data.x
    y_curr = data.y
    th_curr = data.th
    ballx_curr = data.ball_x
    bally_curr = data.ball_y

    rate.sleep()

if __name__ == '__main__':
    main()