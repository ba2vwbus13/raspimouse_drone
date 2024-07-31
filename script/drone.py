import math
import time
import cv2
import numpy as np
import rospy
import rosparam
from std_msgs.msg import String

class DroneControl():
    def __init__(self, img_size):
        self.img_size = img_size
        self.drone_p = self.drone_v = self.target_p = self.target_v = np.array([-1,-1])
        self.movingForward, self.movingAngle = 0.0, 0.0
        self.target_route = self.makeTargetRoute()
        self.target_route_number = 0
        self.recognize = False
        self.prev_time = time.time()
        self.sending_count = 0
        rospy.init_node('talker')
        self.pub = rospy.Publisher('controller', String, queue_size=10)

    def makeTargetRoute(self):
        #target_route = np.array([[0.9,0.1], [0.9,0.9], [0.1,0.9], [0.1,0.1], [-1,-1]])           
        target_route = np.array([[0.9,0.1], [0.9,0.5],[0.1,0.5],[0.1,0.9],[0.9,0.9],
        [0.9,0.5],[0.1,0.5],[0.1,0.1],[0.5,0.1],[0.5,0.9],[0.9,0.9],[0.9,0.1],[0.5,0.1],
        [0.5,0.9],[0.1,0.9],[0.1,0.1], [-1,-1]])       
        return target_route
    
    def getDronePoint(self, detection):       
        points = np.array([[-1.0, -1.0], [-1.0, -1.0]])
        boxes = detection.boxes.cpu().numpy()
        for i, color in enumerate(['green', 'orange']):
            t_accuracy = 0.50 #t_accuracy = 0.75
            for box in boxes:
                id = int(box.cls[0])
                name = detection.names[id]
                accuracy = float(box.conf[0])
                print(name, accuracy)
                if name == color and accuracy > t_accuracy:
                    x1, y1, x2, y2 = box.xyxyn[0]
                    x, y = float((x1+x2)/2), float((y1+y2)/2)
                    t_accuracy = accuracy                             
                    points[i] = np.array([x, y])  
        if np.all(points>0):
            self.recognize = True
            self.drone_p = np.mean(points, axis=0)
            self.drone_v = points[1] - points[0]
        else:
            self.recognize = False

    def getNextTarget(self):
        if np.any(self.target_route[self.target_route_number]<0):
            print("goal!")
            return False
        else:
            self.target_p = self.target_route[self.target_route_number]
            self.target_v = self.target_p - self.drone_p
            target_l = np.linalg.norm(self.target_v, ord=2)
            if target_l < 0.1 and self.recognize:
                print("reach target!")
                self.target_route_number += 1
            return True
            
    def get_drec(self, theta):
        if theta > 90:
            angle = theta/40
            forward = 0
        elif theta > 10:
            angle = theta/30
            forward = 0.03
        else:
            angle = 0
            forward = 0.08
        return forward, angle
    
    def get_drec2(self, theta):
        if theta > 90:
            angle = theta/3
            forward = 0
        elif theta > 10:
            angle = theta/3
            forward = 0.03
        else:
            angle = 0
            forward = 0.08
        return forward, angle
        
    def getMovingDerection(self):
        ret = self.getNextTarget()
        if ret:
            cross = np.cross(self.drone_v, self.target_v)
            inner = np.inner(self.drone_v, self.target_v)
            #outer = np.outer(drone_v, target_v)
            drone_l = np.linalg.norm(self.drone_v, ord=2)
            target_l = np.linalg.norm(self.target_v, ord=2)
            vector_cos = inner / (drone_l * target_l)
            #theta = math.degrees(np.arccos(vector_cos))
            theta = np.rad2deg(np.arccos(np.clip(vector_cos, -1.0, 1.0)))
            forward, angle = self.get_drec(theta)
            if cross > 0:
                angle *= -1
            self.movingForward, self.movingAngle = forward, angle
            return True
        else:
            self.movingForward = 0.0
            self.movingAngle = 0.0
            return False
        
    def sendCommand(self):
        duration = time.time() - self.prev_time
        print(duration)
        if 0.1 < duration <= 2 and self.recognize:
            str = "{},{}".format(self.movingForward, self.movingAngle)
            self.sending_count += 1
        else:
            self.movingForward, self.movingAngle = 0.0, 0.0
            str = "{},{}".format(self.movingForward, self.movingAngle)
        self.prev_time = time.time()

        rospy.loginfo(str)
        self.pub.publish(str)

    def OverImage(self, img):
        yellow, green, blue= (0,255,255), (255,0,255), (255,0,0)
        cyan, magenta, white = (255,255,0), (255,0,255), (255, 255, 255)

        target_p = (self.target_p * self.img_size).astype(int)
        target_v = (self.target_v * self.img_size).astype(int)
        target_v = (target_v / np.linalg.norm(target_v) * 100).astype(int)

        drone_p = (self.drone_p * self.img_size).astype(int)
        drone_v = (self.drone_v * self.img_size).astype(int)
        drove_v = (drone_v / np.linalg.norm(drone_v) * 100).astype(int)

        cv2.circle(img, tuple(target_p), 30, blue, thickness=-1) #target point
        if self.recognize:
            cv2.arrowedLine(img, tuple(drone_p), tuple(drone_v+drone_p), white, thickness=5)#drone direction
            cv2.arrowedLine(img, tuple(drone_p), tuple(target_v+drone_p), magenta, thickness=5)#target direction
        return img