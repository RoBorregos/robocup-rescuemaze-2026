import cv2
from ultralytics import YOLO
from pathlib import Path


BASE_DIR = Path(__file__).resolve().parent
MODEL_PATH = BASE_DIR / "weights" / "best.pt"

model = YOLO(str(MODEL_PATH))

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    results = model.predict(
        frame,
        conf=0.1,
        iou=0.0,
        imgsz=640,
        agnostic_nms=True,
        verbose=False
    )

    annotated_frame = results[0].plot()
    cv2.imshow("Detector", annotated_frame)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()