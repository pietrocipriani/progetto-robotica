from ultralytics import YOLO
import sys
import cv2

def main():
    # model setup
    model = YOLO("runs/detect/multiclass_700/weights/best.pt")

    # show image
    res = model(sys.argv[1], show=True, imgsz=640, line_width=1,
        conf=float(sys.argv[2]), iou=float(sys.argv[3]) if len(sys.argv) > 3 else 0.0)

    # for ((x1, y1, x2, y2), label) in zip(res[0].boxes.xyxy, res[0].boxes.cls):
    #     print(x1.item(),y1.item(),x2.item(),y2.item(),label.item(),res[0].names[int(label)])
    # print(res[0].boxes.xyxy)
    # print(res[0].boxes.cls)

    while True:
        try:
            k = cv2.waitKey(1) & 0xFF
            if k == 27:# or cv2.getWindowProperty(sys.argv[1], 0) < 0:
                break
        except cv2.error:
            break

if __name__ == "__main__":
    main()