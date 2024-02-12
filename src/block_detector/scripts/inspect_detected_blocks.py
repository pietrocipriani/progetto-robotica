#!/usr/bin/env python3

"""
This is a testing script that reads images from `/ur5/zed_node/left/image_rect_color` and passes
them to the `detect_blocks` service (see `block_detector_server.py`) to recognize blocks.
The image is cropped to `[396:912, 676:1544, :]` so that only the workspace is shown.
Finally, a `cv2` window is opened and updated regularly with the results of detection.

Since the detection is quite fast but uses a lot of CPU, if you want to reduce CPU usage when
running this script, you can pass in a number >= 1 to specify one every how many images received
from the zed camera to actually pass to the detection service.
"""

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
    """
    Draws a bounding box on the image using `torchvision.utils.draw_bounding_boxes`.

    @param image the image to copy and then draw on
    @param bbox an array with the bounding boxes to draw, in xyxy format
                (e.g. [(0,1,2,3), (0,2,6,7)])
    @param txt_labels (optional) an array of the same length as `bbox`, with the labels to draw
                alongside the boxes
    """
    with torch.no_grad():
        x = (torchvision.transforms.functional.to_tensor(image) * 255.).to(torch.uint8)
        bbox = bbox.to(torch.int)
        res = torchvision.utils.draw_bounding_boxes(x, bbox, txt_labels, width=1)
        return res.permute(1,2,0).cpu().numpy()


class BlockInspector:
    def __init__(self, skip_period: int):
        """
        Initializes `BlockInspector` and subscribes to the `IMAGE_TOPIC` topic to listen for images.

        @param skip_period only one of this many images will be sent to the block detection service,
                           to save CPU. Setting this to 1 will send all images to the service. 
        """
        self.image_sub = rospy.Subscriber(IMAGE_TOPIC, Image, self.callback, queue_size=1)
        self.encoded_image = None
        self.skip_period = skip_period
        self.counter = skip_period - 1

    def callback(self, encoded_image: Image):
        """
        The callback from the image topic subscription. Saves passes the encoded image to the main
        thread one every `self.skip_period` times, to save CPU.
        """
        self.counter += 1
        if self.counter == self.skip_period:
            self.counter = 0
            # skip N-1 images out of N to save cpu
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
        """
        A non-standalone function to run in a thread, that decodes the image passed as argument,
        sends it to the block detection service, waits for a response, and saves it back
        in a local variable.

        @param encoded_image the bgr8 encoded image to process
        """
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
            # calling `detect_blocks_srv` is blocking, so we need a thread that just receives
            # images (created by BlockInspector when subscribing to the image topic and
            # automatically handled by ros), another thread that processes images (see
            # `process_image`) and finally the main thread for visualization
            if thread is None or not thread.is_alive():
                encoded_image, proc.encoded_image = proc.encoded_image, None
                if encoded_image is not None:
                    thread = threading.Thread(target=process_image, name="process_image", args=[encoded_image])
                    thread.start()

            # visualize the image with the detected blocks using cv2
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