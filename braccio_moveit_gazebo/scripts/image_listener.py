#! /usr/bin/env python3
# Copyright (c) 2015, Rethink Robotics, Inc.

# Using this CvBridge Tutorial for converting
# ROS images to OpenCV2 images
# http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

# Using this OpenCV2 tutorial for saving Images:
# http://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_gui/py_image_display/py_image_display.html

# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2

# Instantiate CvBridge

class CameraShooter(object):
    def __init__(self):
        self.bridge = CvBridge()
        # rospy.init_node('image_listener', anonymous=True)


    def image_callback(self,msg):
        # print("Received an image!")
        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        else:
            # Save your OpenCV2 image as a jpeg 
            cv2.imwrite('camera_image.jpg', cv2_img)


    def shoot(self):
        rate = rospy.Rate(1)
        # Define your image topic
        image_topic = "/camera/color/image_raw"
        # Set up your subscriber and define its callback
        shooter = rospy.Subscriber(image_topic, Image, self.image_callback)
        rate.sleep()
        shooter.unregister()
        # rate.sleep()

        
    # Spin until ctrl + c
    # rospy.spin()

# def main():
#    image_shoot()

# if __name__ == '__main__':
#     main()