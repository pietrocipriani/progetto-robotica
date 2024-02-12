#!/usr/bin/env python3

"""@package block_detector_server
Starts a ROS service that takes an `sensor_msgs/Image` as input, detects boxes in the image, and
returns `block_detector/DetectBlocks`, i.e. the boxes detected by YOLOv8, as output. A service was
preferred, instead of a publisher reading directly from the camera and publishing boxes from time
to time, for the following reasons:
- to allow making this component more standalone
- to allow the input image to be chosen by the caller, e.g. by cropping it or passing a custom image
- to avoid wasting CPU on calculating boxes that would never be used because the subscriber (if any)
    does not need them

This file also contains the configuration to pass to the YOLOv8 model for detecting boxes, such as
CONFIDENCE_THRESHOLD and INTERSECTION_OVER_UNION_THRESHOLD.

You can visualize results from this service using `inspect_detected_blocks.py`.
"""

import rospy
import rospkg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from block_detector.msg import BlockDetectorBox
from block_detector.srv import DetectBlocks, DetectBlocksResponse

import ultralytics
import os

SERVICE_NAME = "detect_blocks"
CONFIDENCE_THRESHOLD = 0.5
INTERSECTION_OVER_UNION_THRESHOLD = 0.5


class BlockDetector:
    def __init__(self):
        """
        Creates an instance of BlockDetector and loads the YOLO model from
        `block_detector_model/model.pt` by resolving the path with `RosPack`.
        """

        self.bridge = CvBridge()

        rospack = rospkg.RosPack()
        model_path = os.path.join(rospack.get_path("block_detector"), "block_detector_model")
        rospy.loginfo(f"block_detector loading model from {model_path}")

        self.model = ultralytics.YOLO(os.path.join(model_path, "model.pt"))
        rospy.loginfo("block_detector done loading model")


    # the service callback, i.e. a function that takes an Image message
    # and returns 
    def callback(self, request: Image):
        """
        The ROS service callback; scripts can call it upon request using `rospy.ServiceProxy`.
        @param request an `sensor_msgs/Image` with bgr8 encoding that will be decoded using the
                       cv2 bridge
        @return `block_detector/DetectBlocks`, i.e. the boxes detected by YOLO
        """

        # decode the image to be compatible with python math libraries
        decoded_image = self.bridge.imgmsg_to_cv2(request.image, desired_encoding="bgr8")

        # the model was trained with imgsz = 640
        result = self.model(
            decoded_image,
            imgsz=640,
            conf=CONFIDENCE_THRESHOLD,
            iou=INTERSECTION_OVER_UNION_THRESHOLD,
        )[0]

        rospy.loginfo(f"block_detector boxes {result.boxes.xyxy}")
        return DetectBlocksResponse(
            boxes = [
                BlockDetectorBox(
                    x1=x1.item(),
                    y1=y1.item(),
                    x2=x2.item(),
                    y2=y2.item(),
                    label=result.names[int(label)],
                    confidence=conf,
                )
                for ((x1, y1, x2, y2), label, conf) in zip(
                    result.boxes.xyxy,
                    result.boxes.cls,
                    result.boxes.conf,
                )
            ]
        )



def main():
    rospy.init_node("block_detector")
    rospy.loginfo("block_detector init")
    proc = BlockDetector()

    # announce the service
    rospy.Service(SERVICE_NAME, DetectBlocks, proc.callback)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass