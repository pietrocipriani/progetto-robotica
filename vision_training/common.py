"""!
This file is undocumented, as Yolos was not used for the project in the end, but YOLOv8 was used
instead.
"""

import torchvision
from matplotlib import pyplot as plt
import numpy as np
import torch
import json


def get_id2label_from_annotations_file(path='assigns/train_annotations.coco.json'):
    with open(path) as f:
        data = json.load(f)

    id2label = {}
    for c in data["categories"]:
        assert c["id"] == len(id2label)
        id2label[len(id2label)] = c["name"]

    # label2id = { v: i for i,v in id2label.items() }
    return id2label

def show_images(*images, cols=2):
    if len(images) == 0:
        return
    if len(images) == 1:
        plt.imshow(images[0])
        plt.show()
        return
    fig = plt.figure()
    for (i, image) in enumerate(images):
        fig.add_subplot((len(images) + cols - 1) // cols, cols, i+1)
        plt.imshow(image)
        plt.axis("off")
        plt.title(f"{i}")
    plt.subplots_adjust(left=0, bottom=0, right=1, top=1, wspace=0.05, hspace=0.05)
    plt.show()

# Helper funciton to draw the bounding boxes
def draw_bbox(image, bbox, txt_labels=None):
    x = (torchvision.transforms.functional.to_tensor(image) * 255.).to(torch.uint8)
    bbox = bbox.to(torch.int)
    res = torchvision.utils.draw_bounding_boxes(x, bbox, txt_labels)
    return res.permute(1,2,0).cpu().numpy()


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
