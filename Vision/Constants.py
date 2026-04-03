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
# Per-camera override (useful for dual IMX219 setups).
# RIGHT (example IMX219 fisheye): middle point between full sensor and half-res.
# Keeps 4:3 and usually looks more natural than 1640x1232.
vision_picamera_right_width = 1920
vision_picamera_right_height = 1440
# LEFT (same IMX219 profile for consistency).
vision_picamera_left_width = 1920
vision_picamera_left_height = 1440
vision_picamera_prefer_full_fov = True
# Preferred Picamera2 main stream format (RGB888 recommended for CV pipeline).
vision_picamera_main_format = "RGB888"
# Color order returned by Picamera2 frames before CV processing.
# Use "BGR" if image looks blue-tinted, "RGB" if colors look correct already.
vision_picamera_color_order = "BGR"
# Optional Picamera2 tuning file. Leave empty to use sensor default tuning.
vision_picamera_tuning_file = ""
# Per-camera color correction before inference (BGR channel gains + HSV tuning).
# RIGHT camera (0): neutral defaults to avoid pink/magenta cast.
vision_right_gain_b = 1.00
vision_right_gain_g = 1.00
vision_right_gain_r = 1.00
vision_right_hue_shift = 0.0
vision_right_saturation_scale = 1.00
# LEFT camera (1): neutral defaults.
vision_left_gain_b = 1.00
vision_left_gain_g = 1.00
vision_left_gain_r = 1.00
vision_left_hue_shift = 0.0
vision_left_saturation_scale = 1.00

# Bullseye geometry tuning (ROI + circle permissiveness for fisheye lenses).
# Global defaults:
target_fast_mode = True
target_verbose = False
target_roi_pad_ratio = 0.10
target_roi_min_pad = 8
target_roi_force_square = True
target_circle_min_radius_ratio = 0.10
target_circle_min_circularity = 0.30
target_circle_contour_min_area = 250.0
target_circle_border_allow_ratio = 0.08
target_ring_n_angles = 56
target_ring_min_radial_samples = 4
target_ring_radial_divisor = 3

# RIGHT camera (0) override: a bit more permissive.
target_right_roi_pad_ratio = 0.15
target_right_circle_min_circularity = 0.24
target_right_circle_border_allow_ratio = 0.14
target_right_ring_n_angles = 48
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
