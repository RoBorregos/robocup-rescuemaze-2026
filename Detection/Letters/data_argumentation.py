from ultralytics import YOLO

model = YOLO('yolov8n.pt') 
model.train(
    data='/Users/marcoalejandrogalindo/Downloads/YOLO_HSU/data.yaml',
    epochs=150,           # Enough for it to learn the curves of the S
    imgsz=640,            # Resolution that yolo works 
    batch=16,             # Number of photos per group
    device='mps',         # my mac gpu
    
    # Data Atgumentation
    degrees=15.0,         # Rotation degrees
    flipud=0.0,           # Don't flip upside down (would confuse the U)
    fliplr=0.5,           # Horizontal flip
    mosaic=1.0,           # Mix 4 photos into one
    mixup=0.1,            # Mix 2 photos together
    
    # Precision Adjustments
    label_smoothing=0.1,  # Helps it not be "stubborn" and learn better
    cls=2.0,              # Gives more importance to not confusing H with S or U
    close_mosaic=10,      # At the end trains with real photos
    name='train5' # Name of the output folder
)
