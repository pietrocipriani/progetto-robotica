"""!
This file is undocumented, as Yolos was not used for the project in the end, but YOLOv8 was used
instead.
"""

import torch
import transformers
import glob
import PIL
import sys
from common import get_id2label_from_annotations_file, show_images, non_max_suppression, draw_bbox
torch.set_float32_matmul_precision('highest')


def make_prediction(image, proc, net, id2label, threshold, overlapThresh):
    """
    image: PIL.Image
    proc: image_processor
    net: torch.nn.Module
    id2label: dict
    """

    inputs = proc(images=image, return_tensors="pt")
    outputs = net(**inputs)

    # convert outputs (bounding boxes and class logits) to COCO API
    target_sizes = torch.tensor([image.size[::-1]])
    results = proc.post_process_object_detection(outputs, threshold=threshold, target_sizes=target_sizes)[0]

    if overlapThresh is None:
        boxes = results["boxes"]
        results = list(zip(results["scores"], results["labels"], results["boxes"]))
    else:
        boxes, results = non_max_suppression(
            boxes=results["boxes"],
            connected_data=list(zip(results["scores"], results["labels"], results["boxes"])),
            overlapThresh=overlapThresh,
        )

    for score, label, box in results:
        box = [round(i, 2) for i in box.tolist()]
        print(
            f"Detected {id2label[label.item()]} with confidence "
            f"{round(score.item(), 3)} at location {box}"
        )

    txt_labels = [id2label[result[1].item()] for result in results]
    pred_on_image = draw_bbox(image, boxes, txt_labels)
    return PIL.Image.fromarray(pred_on_image)


HUGGINGFACE_PATH = "hustvl/yolos-small"

def main():
    feature_extractor = transformers.YolosImageProcessor.from_pretrained(
        HUGGINGFACE_PATH,
        size={"width": 1024, "height": 768},
    )

    # setup id2label
    id2label = get_id2label_from_annotations_file()
    print("FOUND", len(id2label), "LABELS")

    # model setup
    epoch = sorted(glob.glob("./checkpoints/epoch_*"), key=lambda e: int(e.split("_")[-1]))[-1]
    model_inner = transformers.YolosForObjectDetection.from_pretrained(
        epoch,
        num_labels=len(id2label),
    )

    # show image
    image = PIL.Image.open(sys.argv[1])
    pred = make_prediction(image, feature_extractor, model_inner, id2label,
        threshold=float(sys.argv[2]), overlapThresh=float(sys.argv[3]) if len(sys.argv) > 3 else None)
    bbox = PIL.Image.open(sys.argv[1].replace(".jpeg", "_bbox.jpeg"))
    show_images(image, pred, bbox)

if __name__ == "__main__":
    with torch.no_grad():
        main()
