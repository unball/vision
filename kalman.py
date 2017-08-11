#!/usr/bin/env python

import rospy

from vision.msg import VisionMessage


def subscriber(msg):
    
    rate = rospy.Rate(10)

   
    msg = VisionMessage()
    x_curr = msg.x
    y_curr = msg.y
    th_curr = msg.th
    print ('asdasd {0}'.format(type(th_curr)))

    rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('kalman_topic', anonymous=True)
        while not rospy.is_shutdown():
            pub = rospy.Subscriber('vision_topic', VisionMessage, subscriber)
    except rospy.ROSInterruptException:
        pass