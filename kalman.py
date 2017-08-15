#!/usr/bin/env python

import rospy

from vision.msg import VisionMessage

def main():
    rospy.init_node('kalman_topic', anonymous=True)

    while not rospy.is_shutdown():
        rospy.Subscriber('vision_topic', VisionMessage, subscriber, queue_size=1)
        # rospy.spin()
    pass


def subscriber(msg):
    
    rate = rospy.Rate(10)

   
    msg = VisionMessage()
    x_curr = msg.x
    y_curr = msg.y
    th_curr = msg.th
    ballx_curr = msg.ball_x
    bally_curr = msg.ball_y

    print x_curr
    print y_curr
    print bally_curr


    rate.sleep()

if __name__ == '__main__':
    main()