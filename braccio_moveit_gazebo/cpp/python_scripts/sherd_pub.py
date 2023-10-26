import rospy
import numpy as np

from custom_msgs.msg import for_cpp
from custom_msgs.msg import matrix

# if __name__ == '__main__':
class SherdPublisher(object):
    def __init__(self, txt_path):
        rospy.init_node('sherd_pub')

        self.sherd_pub = rospy.Publisher("/sherd_pub", for_cpp, queue_size=10)
        self.mat_pub = rospy.Publisher("/matrix_pub", matrix, queue_size=10)

        rospy.loginfo("sherd_publisher started")
        rospy.loginfo("matrix_publisher started")
        
        with open (txt_path) as cpp:
            pub_array = np.array([[x for x in line.split()] for line in cpp])
            pub_sherd = pub_array[:,0]
            pub_homes = pub_array[:,1]
            print(pub_sherd)
            print(pub_homes)
        self.msg1 = for_cpp()
        self.msg1.sherds = pub_sherd
        self.msg1.homes = pub_homes
        self.rate = rospy.Rate(5)

    def array_pub(self):
        while not rospy.is_shutdown():
            rospy.loginfo(self.msg1)
            self.sherd_pub.publish(self.msg1)
            self.rate.sleep()
        
    def matrix_pub(self):
        while not rospy.is_shutdown():
            msg2 = matrix()
            msg2.dati = [self.msg1.sherds, self.msg1.homes]
            rospy.loginfo(msg2)
            self.mat_pub.publish(msg2)
            self.rate.sleep()
