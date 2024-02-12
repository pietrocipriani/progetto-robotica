#!/usr/bin/env python3
import rospy
import rospkg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from block_detector.msg import BlockDetectorBox
from block_detector.srv import DetectBlocks, DetectBlocksResponse
import torchvision
import torch
import cv2
import threading
import sys

WINDOW_TITLE = "Detected blocks"
IMAGE_TOPIC = "/ur5/zed_node/left/image_rect_color"


def draw_bbox(image, bbox, txt_labels=None):
    with torch.no_grad():
        x = (torchvision.transforms.functional.to_tensor(image) * 255.).to(torch.uint8)
        bbox = bbox.to(torch.int)
        res = torchvision.utils.draw_bounding_boxes(x, bbox, txt_labels, width=1)
        return res.permute(1,2,0).cpu().numpy()


class BlockInspector:
    def __init__(self, skip_period: int):
        self.image_sub = rospy.Subscriber(IMAGE_TOPIC, Image, self.callback, queue_size=1)
        self.encoded_image = None
        self.skip_period = skip_period
        self.counter = skip_period - 1

    def callback(self, encoded_image: Image):
        self.counter += 1
        if self.counter == self.skip_period:
            self.counter = 0
            # skip 4 images out of 5 to save cpu
            self.encoded_image = encoded_image


def main():
    rospy.init_node("inspect_detected_blocks", anonymous=True)
    rospy.loginfo("inspect_detected_blocks init")
    detect_blocks_srv = rospy.ServiceProxy("detect_blocks", DetectBlocks)
    bridge = CvBridge()
    proc = BlockInspector(skip_period = 1 if len(sys.argv) < 2 else int(sys.argv[1]))
    annotated_image = None
    thread = None

    def process_image(encoded_image):
        decoded_image = bridge.imgmsg_to_cv2(encoded_image, desired_encoding="bgr8")
        decoded_image = decoded_image[396:912, 676:1544, :]
        resp = detect_blocks_srv(bridge.cv2_to_imgmsg(decoded_image, encoding="bgr8"))
        data = resp.boxes

        boxes = torch.asarray([[0,0,0,0]] + [[b.x1, b.y1, b.x2, b.y2] for b in data])[1:]
        rospy.loginfo(f"received boxes {boxes}")
        nonlocal annotated_image
        annotated_image = draw_bbox(decoded_image, boxes, [f"{b.label} {b.confidence:.2f}" for b in data])

    try:
        while True:
            if thread is None or not thread.is_alive():
                encoded_image, proc.encoded_image = proc.encoded_image, None
                if encoded_image is not None:
                    thread = threading.Thread(target=process_image, name="process_image", args=[encoded_image])
                    thread.start()

            if annotated_image is not None:
                cv2.imshow(WINDOW_TITLE, annotated_image)
                k = cv2.waitKey(1) & 0xFF
                try:
                    if k == 27 or cv2.getWindowProperty(WINDOW_TITLE, 0) < 0:
                        break
                except cv2.error:
                    print("cv2.error, shutting down")
    except KeyboardInterrupt:
        print("KeyboardInterrupt, shutting down")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass