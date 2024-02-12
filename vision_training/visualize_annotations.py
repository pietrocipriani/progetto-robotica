"""!
Helper to visualize annotations in the COCO Json format. Allows checking if labels and boxes are
correct.
"""

import json
import PIL
import torch
from common import show_images, draw_bbox

import signal
signal.signal(signal.SIGINT, signal.SIG_DFL) # exit on Ctrl+C while showing images


ROOT = "assigns/"
TRAIN_PATH = 'assigns/train_annotations.coco.json'
TEST_PATH = 'assigns/test_annotations.coco.json'

class Box:
    def __init__(self, iid, category_id, category, bbox, area):
        self.iid = iid
        self.category_id = category_id
        self.category = category
        self.bbox = bbox
        self.area = area

    def get_torch_bbox(self):
        return [self.bbox[0], self.bbox[1], self.bbox[0]+self.bbox[2], self.bbox[1]+self.bbox[3]]

class Image:
    def __init__(self, iid, path):
        self.iid = iid
        self.path = path
        self.boxes = []

def load_data(path):
    """!
    @param path a `.coco.json` file to load data from
    """
    with open(path) as f:
        data = json.load(f)

    id2label = []
    for c in data["categories"]:
        assert c["id"] == len(id2label)
        id2label.append(c["name"])

    images = []
    for image in data["images"]:
        assert image["id"] == len(images)
        images.append(Image(image["id"], ROOT + image["file_name"]))

    for a in data["annotations"]:
        images[a["image_id"]].boxes.append(
            Box(a["id"], a["category_id"], id2label[a["category_id"]], a["bbox"], a["area"])
        )

    return images

def main(path):
    """!
    Loads data from the provided path and shows it multiple images at a time.

    @param path a `.coco.json` file to load data from
    """
    images = load_data(path)

    atonce = 1
    for i in range(0, len(images)-atonce+1, atonce):
        merged = []
        for j in range(atonce):
            print(images[i+j].iid, end=", ")
            merged.append(
                draw_bbox(
                    PIL.Image.open(images[i+j].path),
                    torch.asarray([b.get_torch_bbox() for b in images[i+j].boxes]),
                    [b.category for b in images[i+j].boxes]
                )
            )
        print()
        show_images(*merged, cols=3)

if __name__ == "__main__":
    main(TRAIN_PATH)
