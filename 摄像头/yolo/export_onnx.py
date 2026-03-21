from ultralytics import YOLO

model =YOLO(model="/home/chairman/test_dachuang/project_folder/best.pt")

model.export(format="onnx")