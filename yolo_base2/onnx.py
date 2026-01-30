from ultralytics import YOLO

model =YOLO(model="/home/chairman/桌面/ultralytics-8.3.96/runs/detect/train/weights/best.pt")

model.export(format="onnx")