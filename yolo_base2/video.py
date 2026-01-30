import cv2 as cv
cap=cv.VideoCapture(0)
if not cap.isOpened():
    print('无法打开摄像头')
    exit()
while True:
    ret,frame=cap.read()
    if not ret:
        print("无法接收帧")
        break
    cv.imshow('capture',frame)
    if cv.waitKey(1) == ord('q'):
        break
cap.release()
cv.destroyAllWindows()