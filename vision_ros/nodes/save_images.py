#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import sys

topic = sys.argv[1]


def showImage(img):
    cv2.imshow('image', img)
    cv2.waitKey(1)

def process_image(msg):
    rospy.loginfo('detect_pump msg')
    try:
        # convert sensor_msgs/Image to OpenCV Image
        bridge = CvBridge()
        orig = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        drawImg = orig
        cv2.imwrite(topic.replace("/", "__") + ".png", drawImg)
        rospy.signal_shutdown("fatto")
        # exit(0)
        # showImage(drawImg)
    except Exception as err:
        rospy.loginfo(f'detect_pump error {err}')
        print(err)

def start_node():
    rospy.init_node('detect_pump')
    rospy.loginfo('detect_pump node started')
    rospy.Subscriber(topic, Image, process_image)
    rospy.spin()

if __name__ == '__main__':
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass