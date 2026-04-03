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
vision_iou_threshold = 0.30
# If True, force camera output size to vision_frame_width/height.
# If False, keep camera native/default mode (recommended for fisheye to avoid crop).
vision_force_frame_size = False
vision_frame_width = 640
vision_frame_height = 480
# Picamera2 preferred preview size when not forcing OpenCV size.
# For IMX219, 1640x1232 usually preserves more FoV than 640x480.
vision_picamera_width = 1640
vision_picamera_height = 1232
vision_picamera_prefer_full_fov = True
# Explicit Picamera2 camera mapping for CSI setups.
# Typical default: RIGHT=0, LEFT=1 (adjust if physically swapped).
vision_picamera_right_index = 0
vision_picamera_left_index = 1
# Prefer Picamera2 directly for CSI cameras when available.
vision_prefer_picamera2 = True
# When Picamera2 fails on a CSI camera, avoid falling back to OpenCV for that
# same camera to prevent unexpected crop/format changes (fisheye case).
vision_disable_opencv_fallback_when_picamera_preferred = True
vision_device = "cpu"
vision_inference_frames = 1
vision_inference_timeout_ms = 180

# Autofocus (Raspberry Camera Module 3)
# Modes: "continuous", "auto", "manual", "off"
camera_autofocus_mode = "continuous"
# For manual mode only (typical range ~0.0 to ~10.0 depending on lens)
camera_lens_position = 1.5
