from ultralytics import YOLO

model = YOLO("yolov8n.pt")
#model = YOLO("runs/detect/train3/weights/last.pt")

results = model.train(data="assigns/ultralytics.yaml", epochs=10000, imgsz=640)