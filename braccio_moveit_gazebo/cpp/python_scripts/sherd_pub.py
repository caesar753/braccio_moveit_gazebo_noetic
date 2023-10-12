#! /usr/bin/env python3

import rospy
import numpy as np

from custom_msgs.msg import for_cpp

if __name__ == '__main__':
    rospy.init_node('sherd_pub')

    sherd_pub = rospy.Publisher("/sherd_pub", for_cpp, queue_size=10)

    rospy.loginfo("sherd_publisher started")

    rate = rospy.Rate(5)

    with open ("for_cpp.txt") as cpp:
        pub_array = np.array([[x for x in line.split()] for line in cpp])
        pub_sherd = pub_array[:,0]
        pub_homes = pub_array[:,1]
        print(pub_sherd)
        print(pub_homes)

    while not rospy.is_shutdown():
        rospy.loginfo("Hello")
        msg = for_cpp()
        # msg.dati = {"a", "b", "c"}
        msg.sherds = pub_sherd
        msg.homes = pub_homes
        sherd_pub.publish(msg)
        rate.sleep()

    # rospy.sleep(1)

    # rospy.loginfo("Exit now")