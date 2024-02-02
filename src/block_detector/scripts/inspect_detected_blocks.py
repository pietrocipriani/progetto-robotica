#!/usr/bin/env python3
import rospy
import rospkg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from block_detector.msg import BlockDetectorBox, DetectedBlocks
import torchvision
import torch
import cv2


PUBLISH_TOPIC = "detected_blocks"
WINDOW_TITLE = "Detected blocks"


def draw_bbox(image, bbox, txt_labels=None):
    x = (torchvision.transforms.functional.to_tensor(image) * 255.).to(torch.uint8)
    bbox = bbox.to(torch.int)
    res = torchvision.utils.draw_bounding_boxes(x, bbox, txt_labels, width=2)
    return res.permute(1,2,0).cpu().numpy()


class BlockInspector:
    def __init__(self):
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber(PUBLISH_TOPIC, DetectedBlocks, self.callback, queue_size=1)
        self.annotated_image = None

    def callback(self, msg: DetectedBlocks):
        try:
            image = self.bridge.imgmsg_to_cv2(msg.image, desired_encoding="passthrough")
        except CvBridgeError as e:
            print(e)
            return

        with torch.no_grad():
            boxes = torch.asarray([[0,0,0,0]] + [[b.x1, b.y1, b.x2, b.y2] for b in msg.boxes])[1:]
            rospy.loginfo(f"received boxes {boxes}")
            self.annotated_image = draw_bbox(image, boxes)


def main():
    rospy.init_node("inspect_detected_blocks", anonymous=True)
    rospy.loginfo("inspect_detected_blocks init")
    proc = BlockInspector()

    try:
        while True:
            if proc.annotated_image is not None:
                cv2.imshow(WINDOW_TITLE, proc.annotated_image)
                k = cv2.waitKey(1) & 0xFF
                if k == 27 or cv2.getWindowProperty(WINDOW_TITLE, 0) < 0:
                    break
    except cv2.error:
        print("cv2.error, shutting down")
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass