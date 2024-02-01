#!/usr/bin/env python3
import rospy
import rospkg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from block_detector.msg import BlockDetectorBox, DetectedBlocks

import numpy as np
import torch
import transformers
import os

FEATURE_EXTRACTOR_HUGGINGFACE_PATH = "hustvl/yolos-small"
FEATURE_EXTRACTOR_SIZE = {"width": 1024, "height": 768}
NUM_LABELS = 1
IMAGE_TOPIC = "/ur5/zed_node/left/image_rect_color"
PUBLISH_TOPIC = "detected_blocks"
CONFIDENCE_THRESHOLD = 0.8
OVERLAP_THRESHOLD = 0.5 # set to None to skip non-max-suppression


# https://www.pyimagesearch.com/2015/02/16/faster-non-maximum-suppression-python/
def non_max_suppression(boxes, connected_data, overlapThresh):
    # if there are no boxes, return an empty list
    if len(boxes) == 0:
        return boxes, []

    # initialize the list of picked indexes
    pick = []

    # grab the coordinates of the bounding boxes
    x1 = boxes[:,0]
    y1 = boxes[:,1]
    x2 = boxes[:,2]
    y2 = boxes[:,3]

    # compute the area of the bounding boxes and sort the bounding
    # boxes by the bottom-right y-coordinate of the bounding box
    area = (x2 - x1 + 1) * (y2 - y1 + 1)
    idxs = np.argsort(y2)

    # keep looping while some indexes still remain in the indexes
    # list
    while len(idxs) > 0:
        # grab the last index in the indexes list and add the
        # index value to the list of picked indexes
        i = idxs[-1]
        pick.append(i)

        # find the largest (x, y) coordinates for the start of
        # the bounding box and the smallest (x, y) coordinates
        # for the end of the bounding box
        xx1 = np.maximum(x1[i], x1[idxs[:-1]])
        yy1 = np.maximum(y1[i], y1[idxs[:-1]])
        xx2 = np.minimum(x2[i], x2[idxs[:-1]])
        yy2 = np.minimum(y2[i], y2[idxs[:-1]])

        # compute the width and height of the bounding box
        w = np.maximum(0, xx2 - xx1 + 1)
        h = np.maximum(0, yy2 - yy1 + 1)

        # compute the ratio of overlap
        overlap = (w * h) / area[idxs[:-1]]

        # delete all indexes from the index list that have
        idxs = (idxs[:-1])[overlap < overlapThresh]

    # return only the bounding boxes that were picked using the
    # integer data type
    return boxes[tuple(pick),], list(map(connected_data.__getitem__, pick))


class BlockDetector:
    def __init__(self):
        self.bridge = CvBridge()

        rospack = rospkg.RosPack()
        model_path = os.path.join(rospack.get_path("block_detector"), "block_detector_model")
        rospy.loginfo(f"block_detector loading model from {model_path}")

        with torch.no_grad():
            self.feature_extractor = transformers.YolosImageProcessor.from_pretrained(
                FEATURE_EXTRACTOR_HUGGINGFACE_PATH,
                size=FEATURE_EXTRACTOR_SIZE,
            )
            self.model = transformers.YolosForObjectDetection.from_pretrained(
                model_path,
                num_labels=NUM_LABELS,
            )

        rospy.loginfo("block_detector done loading model")

        self.blocks_pub = rospy.Publisher(PUBLISH_TOPIC, DetectedBlocks, queue_size=1)
        self.image_sub = rospy.Subscriber(IMAGE_TOPIC, Image, self.callback, queue_size=1)


    def callback(self, msg: Image):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except CvBridgeError as e:
            print(e)
            return

        with torch.no_grad():
            inputs = self.feature_extractor(images=image, return_tensors="pt")
            outputs = self.model(**inputs)

            target_sizes = torch.tensor([image.shape[:2]])
            results = self.feature_extractor.post_process_object_detection(
                outputs,
                threshold=CONFIDENCE_THRESHOLD,
                target_sizes=target_sizes
            )[0]

            if OVERLAP_THRESHOLD is None:
                boxes = results["boxes"]
                results = list(zip(results["scores"], results["labels"], results["boxes"]))
            else:
                boxes, results = non_max_suppression(
                    boxes=results["boxes"],
                    connected_data=list(zip(results["scores"], results["labels"], results["boxes"])),
                    overlapThresh=OVERLAP_THRESHOLD,
                )

        rospy.loginfo(f"block_detector boxes {boxes}")
        detected_blocks_msg = DetectedBlocks(
            image=msg,
            source_image_topic=IMAGE_TOPIC,
            boxes=[
                BlockDetectorBox(x1=x1.item(), y1=y1.item(), x2=x2.item(), y2=y2.item())
                for (x1, y1, x2, y2) in boxes
            ]
        )
        self.blocks_pub.publish(detected_blocks_msg)



def main():
    rospy.init_node("block_detector")
    rospy.loginfo("block_detector init")
    proc = BlockDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass