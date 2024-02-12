"""!
Very simple script used to fine-tune a YOLOv8 model on the blocks dataset.
The ultralytics library is really great, and automatically:
- chooses hyperparameters
- performs augmentation
- stops training when there is no improvement
- generates a lot of useful visualizations in the `runs/` folder
- keeps track of the last and the best models
"""

from ultralytics import YOLO

model = YOLO("yolov8n.pt")
#model = YOLO("runs/detect/train3/weights/last.pt")

# automatically stops training when it detects there is no improving anymore
results = model.train(data="assigns/ultralytics.yaml", epochs=10000, imgsz=640)