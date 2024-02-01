#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import torch
import transformers

TOPIC = "/ur5/zed_node/left/image_rect_color"

class ImageProcessor:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(TOPIC, Image, self.callback, queue_size=1)

    def callback(self, msg):
        rospy.loginfo("block_detector msg")
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except CvBridgeError as e:
            print(e)
            return

def main():
    rospy.init_node("block_detector")
    rospy.loginfo("block_detector init")
    proc = ImageProcessor()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass