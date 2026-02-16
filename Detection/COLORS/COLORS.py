import cv2
import numpy as np
import time

def detect_target_color(frame): 
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # Define HSV ranges for the target colors (ignoring White)
    hsv_colors = {
        "Y": ([20, 100, 100], [30, 255, 255]),
        "G": ([35, 50, 50], [85, 255, 255]),
   # Two ranges of red because it will confuse 
        "R": ([0, 120, 70], [10, 255, 255], [170, 120, 70], [180, 255, 255]),
    }
    
    pixel_counts = {}
    for color, values in hsv_colors.items():
        if color == "R":
            mask1 = cv2.inRange(hsv, np.array(values[0]), np.array(values[1]))
            mask2 = cv2.inRange(hsv, np.array(values[2]), np.array(values[3]))
            mask = cv2.bitwise_or(mask1, mask2)
        else:
            mask = cv2.inRange(hsv, np.array(values[0]), np.array(values[1]))
        
        pixel_counts[color] = cv2.countNonZero(mask)
    
    # Find the color with the maximum count
    predominant_color = max(pixel_counts, key=pixel_counts.get)
    max_pixels = pixel_counts[predominant_color]
    
    # Only report the color if it occupies a significant area (like > 500 pixels)
    if max_pixels > 500: 
        return predominant_color
    else:
        return "None"

# CAMERA SETUP
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("ERROR: CAMARA")
    exit()

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.1)
            continue
    
        detected_color = detect_target_color(frame)
        print(f"Detected: {detected_color}")
        cv2.imshow(' q to exit', frame)
        
        # Exit condition
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    cap.release()
    cv2.destroyAllWindows()