# Cameras
# 0 = RIGHT, 1 = LEFT by default (change if needed)
camera_right_index = 0
camera_left_index = 1

# Backward compatibility
camara_index = camera_right_index

# Serial communication settings
serial_port = "/dev/serial0"
# serial_port = "/dev/ttyACM0"
baud_rate = 115200
timeout = 0.5

# Vision settings
vision_conf_threshold = 0.25
vision_imgsz = 640
vision_iou_threshold = 0.50
vision_frame_width = 640
vision_frame_height = 480
vision_device = "cpu"