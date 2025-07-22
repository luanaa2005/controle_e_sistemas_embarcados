#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def talker():
    rospy.init_node("talker_python", anonymous=False)
    loop_rate = rospy.Rate(10)
    chatter_pub = rospy.Publisher("chatter", String, queue_size = 100)

    count = 0
    while not rospy.is_shutdown():
        txt = "Ola ROS Python! Contagem: " + str(count)
        rospy.loginfo(txt)
        chatter_pub.publish(txt)
        loop_rate.sleep()
        count = count + 1

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass