#!/usr/bin/env python3

#あらかじめ実行すること
#(1) ~/RaspberryPiMouse/utils/build_install.bash
#(2) roslaunch raspimouse_ros_2 raspimouse.launch
#ip address 192.168.11.15

import rospy
import math
import numpy as np
from time import sleep
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse

class Drone():
    def __init__(self):
        rospy.init_node('receiver')
        rospy.wait_for_service('/motor_on')
        rospy.wait_for_service('/motor_off')
        rospy.on_shutdown(rospy.ServiceProxy('/motor_off', Trigger).call)
        rospy.ServiceProxy('/motor_on', Trigger).call()
        rospy.Subscriber('controller', String, self.set_twist)
        rospy.Timer(rospy.Duration(0.1), self.safety)
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.data = Twist()
        self.data.linear.x = 0.0
        self.data.angular.z = 0.0
        self.previous_control_tim = 0.0

    def safety(self):
        delta_time = rospy.get_time() - self.previous_control_time
        if delta_time > 0.3:
            self.data.linear.x = 0.0
            self.data.angular.z = 0.0

    def set_twist(self, data):
        #rospy.loginfo("recieved %s", data.data)
        rev = data.data.split(',')
        self.data.linear.x = float(rev[0])
        self.data.angular.z = float(rev[1])
        self.previous_control_time = rospy.get_time()

    def run(self):
        rospy.loginfo("x:%f, z:%f", self.data.linear.x, self.data.angular.z)
        self.cmd_vel.publish(self.data)

if __name__ == '__main__':
    drone = Drone()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        drone.run()
        rate.sleep()