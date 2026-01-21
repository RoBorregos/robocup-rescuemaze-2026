import cv2
from ultralytics import YOLO

# The best model made is (train4)
model = YOLO('/Users/marcoalejandrogalindo/runs/detect/train4/weights/best.pt')

cap = cv2.VideoCapture(0)
 
# Camera resolution because of YOLO performance issues
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

while True:
    ret, frame = cap.read()
    if not ret: break

    # We raise to 0.25 to clean false detections, 
    # but we compensate with 'agnostic_nms' for more precision.
    results = model.predict(
        frame, 
        conf=0.1,          # A bit higher for more cleaning
        iou=0.0,           # Avoids double boxes
        device='mps',       # My mac gpu 
        imgsz=640,          # We force the training resolution
        agnostic_nms=True,  # To not confuse S with U or H
        verbose=False
    ) 

    annotated_frame = results[0].plot()
    cv2.imshow('Detector', annotated_frame)

    # exit'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()

# python /Users/marcoalejandrogalindo//YOLO_HSU/camera_detector.py
# to run this file i use a venv with ultralytics installed
# source /Users/marcoalejandrogalindo/venv_yolo/bin/activate

