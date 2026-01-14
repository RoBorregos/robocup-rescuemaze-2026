import cv2
from ultralytics import YOLO
#Final model path
model_path = '/Users/marcoalejandrogalindo/Vision TMR MAZE/runs/detect/modelo_final_HSU/weights/best.pt'
model = YOLO(model_path)


cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
while True:
    ret, frame = cap.read()
    if not ret: break
    
    results = model.predict(
        frame, 
        conf=0.25,         # Confidence threshold
        device='mps',      # my mac gpu
        iou=0.5,           # clearer boxes
        imgsz=640,         # resolution
        verbose=False
    )
    annotated_frame = results[0].plot()
    
    cv2.imshow('TMR MAZE - Detección HSU Final', annotated_frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
#python "/Users/marcoalejandrogalindo/Vision TMR MAZE/Maze-TMR-VISION-/Detection/HSU/HSU_FINAL.py"
# # to run this file i use a venv with ultralytics installed
# source /Users/marcoalejandrogalindo/venv_yolo/bin/activate
#some times u have to downlawd con tne venv pip install opencv-python ultralytics