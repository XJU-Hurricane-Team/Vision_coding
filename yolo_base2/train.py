from ultralytics import YOLO
model=YOLO("yolo11n.pt")

model.train(data='yolo-vbn.yaml',workers=1,epochs=30,batch=16)

