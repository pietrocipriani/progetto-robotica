import json
import glob
import os
import random
import sys

# the letter mapping is:
# H: all top squares have a circle, the base is high
# L: all top squares have a circle, the base is low
# T: only half of the top squares have a circle, the other part a half-triangle
# U: only half of the top squares have a circle, the other part is U-shaped
VERTICES_TO_CATEGORIES = {
    1971: (0,  "1x1_H"), # assign1/scene1/view=0
    2691: (1,  "2x1_T"), # assign1/scene2/view=0
    2609: (2,  "2x1_L"), # assign1/scene4/view=0
    3536: (3,  "2x1_H"), # assign1/scene13/view=0
    2881: (4,  "2x1_U"), # assign1/scene16/view=0
    6281: (5,  "2x2_H"), # assign1/scene19/view=0
    5306: (6,  "2x2_U"), # assign1/scene3/view=0
    5209: (7,  "3x1_H"), # assign1/scene6/view=0
    3396: (8,  "3x1_U"), # assign1/scene5/view=0
    6716: (9,  "4x1_H"), # assign1/scene11/view=0
    4711: (10, "4x1_L"), # assign1/scene30/view=0
}

MODES = ["with_categories", "without_categories"]


def get_views(root):
    assigns = []
    for i in range(1,4):
        assign = []
        for scenepath in glob.glob(os.path.join(root, f"assign{i}/scene*")):
            scene = []
            for view in glob.glob(os.path.join(scenepath, "view=*.json")):
                scene.append(os.path.relpath(view[:-5], root))
            scene = sorted(scene, key=lambda a: int(a.split("view=")[-1]))
            assign.append(scene)
        assign = sorted(assign, key=lambda a: int(a[0].split("scene")[-1].split("/")[0]))
        assigns.append(assign)
    return assigns

def flatten_2(arr):
    return [c for a in arr for b in a for c in b]

def get_coco_json(root, paths, collapse_to_1_category):
    """
    Set `collapse_to_1_category` to True if you want a single
    category without distinction between blocks classes
    """

    images = []
    annotations = []

    for path in paths:
        image_id = len(images)
        print(image_id, end="\r")
        images.append({
            "id": image_id,
            "file_name": path + ".jpeg",
            "height": 1024,
            "width": 1024,
        })

        data = json.load(open(os.path.join(root, path + ".json")))
        for obj in data.values():
            if collapse_to_1_category:
                category_id = 0
            else:
                # use the number of vertices to understand the corresponding block type
                vh = len(obj["vertices"])
                category_id = VERTICES_TO_CATEGORIES[vh][0]

            x1,y1,x2,y2 = 1024,1024,0,0
            for (x,y) in obj["vertices"]:
                x1 = min(x1, x)
                y1 = min(y1, y)
                x2 = max(x2, x)
                y2 = max(y2, y)

            # uncomment to visualize vertices on an image
            # bbox3d = obj["vertices"]
            # from PIL import Image, ImageDraw
            # from matplotlib import pyplot as plt
            # image = Image.open(os.path.join(root, path + ".jpeg"))
            # draw = ImageDraw.Draw(image, "RGBA")
            # for point in bbox3d:
            #     draw.point(point)
            # plt.imshow(image)
            # plt.show()
            # exit(0)

            annotations.append({
                "id": len(annotations),
                "image_id": image_id,
                "category_id": category_id,
                "bbox": [x1, y1, x2-x1, y2-y1],
                "area": (x2-x1)*(y2-y1),
            })

    if collapse_to_1_category:
        # Treat everything as a single category
        categories = [{"id": 0, "name": ""}]
    else:
        categories = []
        for (vh, (category_id, name)) in VERTICES_TO_CATEGORIES.items():
            if category_id not in [c["id"] for c in categories]:
                categories.append({
                    "id": category_id,
                    "name": name,
                })

    print()
    return {
        "categories": categories,
        "images": images,
        "annotations": annotations,
    }

def split_train_test(paths, train_over_total_ratio, seed):
    rng = random.Random(seed)
    paths = rng.sample(paths, len(paths))
    mid = int(train_over_total_ratio * len(paths))
    return paths[:mid], paths[mid:]

def main():
    if len(sys.argv) == 1 or len(sys.argv) > 2 or sys.argv[1] not in MODES:
        print(f"Usage: {sys.argv[0]} [{'|'.join(MODES)}]")
        exit(1)
    collapse_to_1_category = sys.argv[1] == MODES[1]

    root = "assigns"
    views = get_views(root)
    paths = flatten_2(views)
    train, test = split_train_test(paths, 0.75, 2378)
    print(f"train size: {len(train)}; test size: {len(test)}")

    for prefix, paths in [("train", train), ("test", test)]:
        res = get_coco_json(root, paths, collapse_to_1_category)
        json.dump(res, open(os.path.join(root, prefix + "_annotations.coco.json"), "w"), indent=4)

if __name__ == "__main__":
    main()
