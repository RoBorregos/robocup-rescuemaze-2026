import cv2
import numpy as np

COLORS = {
    "BLUE":   {"id": 2,  "h_range": (100, 130), "min_s": 50},
    "GREEN":  {"id": 1,  "h_range": (40, 95),   "min_s": 30},
 
    "RED":    {"id": -1, "h_range1": (0, 15),   "h_range2": (160, 180), "min_s": 50},
    "YELLOW": {"id": 0,  "h_range": (20, 35),   "min_s": 50},
    "BLACK":  {"id": -2, "max_v": 40}  # White/harmed center reference
}

def get_ring_value(hsv_roi):
    "Analyzes a small region and returns its score value"
    if hsv_roi.size == 0:
        return 0, (127, 127, 127)
    
    # Use median to reduce brightness noise
    h = np.median(hsv_roi[:, :, 0])
    s = np.median(hsv_roi[:, :, 1])
    v = np.median(hsv_roi[:, :, 2])

    # priority: black (center)
    if v < COLORS["BLACK"]["max_v"]:
        return -2, (0, 0, 0)
    if s > 40:
        if COLORS["BLUE"]["h_range"][0] < h < COLORS["BLUE"]["h_range"][1]:
            return 2, (255, 0, 0)
        if COLORS["GREEN"]["h_range"][0] < h < COLORS["GREEN"]["h_range"][1]:
            return 1, (0, 255, 0)
        if (COLORS["RED"]["h_range1"][0] < h < COLORS["RED"]["h_range1"][1]) or \
           (COLORS["RED"]["h_range2"][0] < h < COLORS["RED"]["h_range2"][1]):
            return -1, (0, 0, 255)
        if COLORS["YELLOW"]["h_range"][0] < h < COLORS["YELLOW"]["h_range"][1]:
            return 0, (0, 255, 255)
            
    return 0, (200, 200, 200)  # White or pale yellow


def process_target(frame):
    if frame is None:
        return None
    
    output = frame.copy()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.medianBlur(gray, 7) 
    
    # CIRCLE DETECTION
    circles = cv2.HoughCircles(
        gray,
        cv2.HOUGH_GRADIENT,
        dp=1.2,
        minDist=100,
        param1=50,
        param2=60,
        minRadius=60,
        maxRadius=250
    )

    if circles is not None:
        circles = np.uint16(np.around(circles))
        
        # If multiple circles detected, choose the largest radius
        main_circle = max(circles[0, :], key=lambda x: x[2])
        
        center = (main_circle[0], main_circle[1])
        radius = main_circle[2]
        
        # Draw detected outer border (Green)
        cv2.circle(output, center, radius, (0, 255, 0), 3)
        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        total_sum = 0
        
        # Sample 5 rings (10%, 30%, 50%, 70%, 90% of radius)
        steps = [0.1, 0.3, 0.5, 0.7, 0.9]

        for step in steps:
            sample_x = int(center[0] + (radius * step))
            sample_y = center[1]
            
            # Ensure point is inside frame
            if 5 < sample_x < frame.shape[1] - 5:
                roi = hsv[sample_y-2:sample_y+3, sample_x-2:sample_x+3]
                val, color_bgr = get_ring_value(roi)
                total_sum += val
                
                # Debug point and detected value
                cv2.circle(output, (sample_x, sample_y), 5, color_bgr, -1)
                cv2.circle(output, (sample_x, sample_y), 6, (255, 255, 255), 1)

        status = "UNHARMED"
        text_color = (255, 255, 255)
        
        if total_sum == 2:
            status = "HARMED (2 KITS)"
            text_color = (0, 0, 255)
        elif total_sum == 1:
            status = "STABLE (1 KIT)"
            text_color = (0, 255, 0)
        elif total_sum == 0:
            status = "UNHARMED (0 KITS)"
            text_color = (255, 255, 0)

        # Display information on screen
        cv2.putText(output, f"Sum: {total_sum}", (20, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, text_color, 2)
        cv2.putText(output, status, (20, 90),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, text_color, 2)

    return output


cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    result_img = process_target(frame)
    
    cv2.imshow("Scanner Targets", result_img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
