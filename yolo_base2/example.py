from ultralytics import YOLO
import cv2 as cv

# 加载模型
yolo = YOLO("/home/chairman/working/sum_result/yolo/yolo_base2/runs/detect/train/weights/best.onnx", task='detect')

# 打开摄像头
cap = cv.VideoCapture(0)
if not cap.isOpened():
    print('无法打开摄像头')
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("无法接收帧")
        break

    results = yolo(source=frame, verbose=False, conf=0.35)[0]

    if len(results.boxes.data) == 0:
        print("没有目标物体")
    else:
        for box in results.boxes.data:
            x1, y1, x2, y2, score = box[:5]
            center_x=(x1+x2)/2
            center_y=(y1+y2)/2
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
            print("类别名字：pin    像素坐标：(%.2f,%.2f)"%(center_x,center_y))
            cv.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label = 'pin:%.2f (%.2f,%.2f)'%(score,center_x,center_y)
            cv.putText(frame, label, (x1, y1 - 10), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    cv.imshow('capture', frame)

    if cv.waitKey(10) == ord('q'):
        break

cap.release()
cv.destroyAllWindows()
