from ultralytics import YOLO

model = YOLO("yolov8n.pt")

results = model.train(data="assigns/ultralytics.yaml", epochs=150, imgsz=640)