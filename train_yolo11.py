from ultralytics import YOLO

model = YOLO("yolo11n.yaml") 
model.train(data="dataset_colors/data.yaml", epochs=100, imgsz=640, save_dir="models")