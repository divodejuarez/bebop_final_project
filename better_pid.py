#!/usr/bin/env python

import rospy
import threading
import time
import sys
import math
import numpy as np
import sys
import cv2

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import String 
from std_msgs.msg import Empty 
from std_msgs.msg import UInt8

from math import sin, cos
from simple_pid import PID


class AutonomousFlight():
    def __init__(self):
        self.status = "LAND"
        rospy.init_node('pid_bebop',anonymous=False)
        self.rate           = rospy.Rate(10)

        self.origin         = None
        self.movePoint      = None
        self.currPosition   = None
        
        self.cPostionPy     = None
        self.originPy       = None
        self.movePointPy    = None
        
        self.first          = True

        self.pids           = list()
        self.pid_vels       = list()

        for i in range(3):
            self.pid_vels.append(0)

        self.pubTakeoff     = rospy.Publisher("bebop/takeoff",Empty,queue_size=10)
        self.pubLand        = rospy.Publisher("bebop/land",Empty,queue_size=10)
        self.pubvelocity    = rospy.Publisher("bebop/cmd_vel",Twist,queue_size=10)
        self.pubFlip        = rospy.Publisher("bebop/flip",UInt8,queue_size=10)

        self.velocity       = Twist()
        self.onePoint       = Point()

        self.subOdom        = rospy.Subscriber("bebop/odom",Odometry,self.updatePosition)
        self.pid_daemon     = threading.Thread(target=self.pid_vel_error)
        self.pid_daemon.setDaemon(True)

        # Camera callbacks and variables
        # self.bridge = CvBridge()
        # self.image_raw = rospy.Subscriber("bebop/image_raw", Image, self.process_image)

        rospy.on_shutdown(self.send_land)
    
    def updatePosition(self, odom):
        if self.first:
            self.origin = odom.pose.pose.position
            self.originPy = RosPoint(self.origin)
            self.currPosition = self.origin
            self.cPostionPy = RosPoint(self.origin)
            self.first = False
            self.movePointPy = RosPoint(self.origin)
            self.pid_daemon.start()
        self.currPosition   = odom.pose.pose.position
        self.cPostionPy.updatePoint(self.currPosition)
        # print('PosX: ' + str(self.cPostionPy.x) + '\t PosY:' + str(self.cPostionPy.y))
    
    # def process_image(self, img):
    #     try:
    #         cv_image = self.bridge.imgmsg_to_cv2(img, "bgr8")
    #     except CvBridgeError as e:
    #         print(e)
        
    #     cv2.imshow("Image Window", cv_image)

    
    def set_velocity(self,linear_x,linear_y,linear_z,angular_x,angular_y,angular_z):
        self.velocity.linear.x   = linear_x #forward (+) backward (-)
        self.velocity.linear.y   = linear_y #left (+) right (-)
        self.velocity.linear.z   = linear_z #up (+) down (-)
        self.velocity.angular.x  = angular_x
        self.velocity.angular.y  = angular_y #clockwise (-) anticlockwise (+)
        self.velocity.angular.z  = angular_z
        self.pubvelocity.publish(self.velocity)
        self.rate.sleep()
    
    def send_take_off(self):
        self.pubTakeoff.publish(Empty())
        print("takeoff")
        self.rate.sleep()
        time.sleep(8)
        self.movePointPy.updatePoint(self.currPosition)
        self.status = 'AIR'
        print('STATUS: ' + self.status + '\n')

    def send_land(self):
        self.pubLand.publish(Empty())
        self.status = 'LAND'
        print("landing")
    
    def pid_vel_error(self):
        #Initialize PIDs & vels
        #For X
        self.pids.append(PID(0.095, 0, 0.01, output_limits=(-0.15,0.15), setpoint=self.movePointPy.x))
        #For Y
        self.pids.append(PID(0.095, 0, 0.01, output_limits=(-0.15,0.15), setpoint=self.movePointPy.y))
        #For Z
        self.pids.append(PID(0.095, 0, 0.01, output_limits=(-0.15,0.15), setpoint=self.movePointPy.z))

        while True:
            if self.status == 'AIR':
                #For x
                self.pids[0].setpoint = self.movePointPy.x
                #For y
                self.pids[1].setpoint = self.movePointPy.y
                #For z
                self.pids[2].setpoint = self.movePointPy.z
                #print('Movepoint x: ' + str(self.movePointPy.x))
            else:
                pass
    
    def moveToX(self, coordinates): #coordinates is given as a list of 3 elements
        self.onePoint.x = coordinates
        self.onePoint.y = 0
        self.onePoint.z = 0
        
        self.movePointPy.addPoint(self.onePoint)

        subPoint = RosPoint(self.movePointPy)

        subPoint.substractPoint(self.cPostionPy)
        # print('error X: ' + str(subPoint.x) + '\t error Y: ' + str(subPoint.y))
        
        while abs(subPoint.x) > 0.1: # or abs(subPoint.y) > 0.1: # or abs(subPoint.z) > 0.1:
            #Get the vels given the pids
            self.pid_vels[0] = self.pids[0](self.cPostionPy.x)
            # self.pid_vels[1] = self.pids[1](self.cPostionPy.y)
            # self.pid_vels[2] = self.pids[2](self.cPostionPy.z)
            # print('pid X: ' + str(self.pid_vels[0]) + ' - pid Y: ' + str(self.pid_vels[1]) + ' - pid Z: ' + str(self.pid_vels[2]))
            # self.set_velocity(self.pid_vels[0],self.pid_vels[1],self.pid_vels[2],0,0,0)
            self.set_velocity(self.pid_vels[0],0,0,0,0,0)
            subPoint.updatePoint(self.movePointPy)
            subPoint.substractPoint(self.cPostionPy)
            # print('error X: ' + str(abs(subPoint.x)) + '\t error Y: ' + str(abs(subPoint.y)))

        print('Finished... X')
    
    def moveToY(self, coordinates):
        self.onePoint.x = 0
        self.onePoint.y = coordinates
        self.onePoint.z = 0
        
        self.movePointPy.addPoint(self.onePoint)

        subPoint = RosPoint(self.movePointPy)

        subPoint.substractPoint(self.cPostionPy)
        # print('error X: ' + str(subPoint.x) + '\t error Y: ' + str(subPoint.y))
        
        while abs(subPoint.y) > 0.1: # or abs(subPoint.y) > 0.1: # or abs(subPoint.z) > 0.1:
            #Get the vels given the pids
            # self.pid_vels[0] = self.pids[0](self.cPostionPy.x)
            self.pid_vels[1] = self.pids[1](self.cPostionPy.y)
            # self.pid_vels[2] = self.pids[2](self.cPostionPy.z)
            # print('pid X: ' + str(self.pid_vels[0]) + ' - pid Y: ' + str(self.pid_vels[1]) + ' - pid Z: ' + str(self.pid_vels[2]))
            # self.set_velocity(self.pid_vels[0],self.pid_vels[1],self.pid_vels[2],0,0,0)
            self.set_velocity(0,self.pid_vels[1],0,0,0,0)
            subPoint.updatePoint(self.movePointPy)
            subPoint.substractPoint(self.cPostionPy)
            # print('error X: ' + str(abs(subPoint.x)) + '\t error Y: ' + str(abs(subPoint.y)))

        print('Finished... Y')

    def stay(self, T):
	    self.set_velocity(0,0,0,0,0,0)
	    time.sleep(T)

class RosPoint():
    def __init__(self,Point):
        self.x = Point.x
        self.y = Point.y
        self.z = Point.z
    
    def addPoint(self, Point):
        self.x += Point.x
        self.y += Point.y
        self.z += Point.z
    
    def updatePoint(self, Point):
        self.x = Point.x
        self.y = Point.y
        self.z = Point.z
    
    def substractPoint(self, Point):
        self.x -= Point.x
        self.y -= Point.y
        self.z -= Point.z
    

if __name__ == '__main__':
    try:
        uav = AutonomousFlight()
        count = 0
        while not rospy.is_shutdown():
            uav.send_take_off()
            uav.stay(0)
            if count == 2:
                uav.moveToX(1.5)
                uav.stay(1)
            if count == 3:
                uav.moveToY(-1.0)
                uav.stay(1)
                uav.send_land()
                sys.exit()
            count += 1

    except rospy.ROSInterruptException:
        pass