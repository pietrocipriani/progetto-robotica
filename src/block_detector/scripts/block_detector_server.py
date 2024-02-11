#!/usr/bin/env python3
import rospy
import rospkg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from block_detector.msg import BlockDetectorBox
from block_detector.srv import DetectBlocks, DetectBlocksResponse

import ultralytics
import os

NUM_LABELS = 1
PUBLISH_TOPIC = "detected_blocks"
CONFIDENCE_THRESHOLD = 0.5
INTERSECTION_OVER_UNION_THRESHOLD = 0.5


class BlockDetector:
    def __init__(self):
        self.bridge = CvBridge()

        rospack = rospkg.RosPack()
        model_path = os.path.join(rospack.get_path("block_detector"), "block_detector_model")
        rospy.loginfo(f"block_detector loading model from {model_path}")

        self.model = ultralytics.YOLO(os.path.join(model_path, "model.pt"))

        rospy.loginfo("block_detector done loading model")


    def callback(self, request: Image):
        decoded_image = self.bridge.imgmsg_to_cv2(request.image, desired_encoding="bgr8")

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
    rospy.Service("detect_blocks", DetectBlocks, proc.callback)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass