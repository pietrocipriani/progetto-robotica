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


WINDOW_TITLE = "Detected blocks"
IMAGE_TOPIC = "/ur5/zed_node/left/image_rect_color"


def draw_bbox(image, bbox, txt_labels=None):
    x = (torchvision.transforms.functional.to_tensor(image) * 255.).to(torch.uint8)
    bbox = bbox.to(torch.int)
    res = torchvision.utils.draw_bounding_boxes(x, bbox, txt_labels, width=1)
    return res.permute(1,2,0).cpu().numpy()


class BlockInspector:
    def __init__(self, detect_blocks_srv):
        self.bridge = CvBridge()
        self.detect_blocks_srv = detect_blocks_srv
        self.image_sub = rospy.Subscriber(IMAGE_TOPIC, Image, self.callback, queue_size=1)
        self.annotated_image = None

    def callback(self, encoded_image: Image):
        try:
            decoded_image = self.bridge.imgmsg_to_cv2(encoded_image, desired_encoding="bgr8")
            decoded_image = decoded_image[396:912, 676:1544, :]
        except CvBridgeError as e:
            print(e)
            return

        resp = self.detect_blocks_srv(self.bridge.cv2_to_imgmsg(decoded_image, encoding="bgr8"))
        data = resp.boxes

        with torch.no_grad():
            boxes = torch.asarray([[0,0,0,0]] + [[b.x1, b.y1, b.x2, b.y2] for b in data])[1:]
            rospy.loginfo(f"received boxes {boxes}")
            self.annotated_image = draw_bbox(decoded_image, boxes, [b.label for b in data])


def main():
    rospy.init_node("inspect_detected_blocks", anonymous=True)
    rospy.loginfo("inspect_detected_blocks init")
    detect_blocks_srv = rospy.ServiceProxy("detect_blocks", DetectBlocks)
    proc = BlockInspector(detect_blocks_srv)

    try:
        while True:
            if proc.annotated_image is not None:
                cv2.imshow(WINDOW_TITLE, proc.annotated_image)
                k = cv2.waitKey(1) & 0xFF
                try:
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