#!/usr/bin/env python3
import cv2
import numpy as np
from ultralytics import YOLO
import rospy
import drone

# ウェブカメラのキャプチャを開始
video_path = 0
cap = cv2.VideoCapture(video_path)
img_size = np.array([cap.get(cv2.CAP_PROP_FRAME_WIDTH), cap.get(cv2.CAP_PROP_FRAME_HEIGHT)])

# Load a pretrained YOLOv8n model
model_path = "/home/nakahira/workspace/procon2024/BallDetect/runs/detect/train7/weights/best.pt"
model = YOLO(model_path)

drone = drone.DroneControl(img_size)
#drone = DroneControl()

rate = rospy.Rate(10)
# キャプチャがオープンしている間続ける
while(cap.isOpened() and not rospy.is_shutdown()):
    # フレームを読み込む
    ret, frame = cap.read()
    if ret == True:
        # フレームを表示
        detections = model(frame)
        detection = detections[0]
        drone.getDronePoint(detection)
        drone.getMovingDerection()
        frame = detection.plot()
        frame = drone.OverImage(frame)
        #frame = cvDroneInfo(drone, frame)
        drone.sendCommand()
        cv2.imshow('Webcam Live', frame)
        rate.sleep()
        # 'q'キーが押されたらループから抜ける
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    else:
        break

# キャプチャをリリースし、ウィンドウを閉じる
cap.release()
cv2.destroyAllWindows()