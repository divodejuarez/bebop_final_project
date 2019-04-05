#!/usr/bin/env python

import rospy 
import time
import sys
import math
import numpy as np

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from std_msgs.msg import String 
from std_msgs.msg import Empty 
from std_msgs.msg import UInt8

from math import sin, cos
from simple_pid import PID

COMMAND_PERIOD = 1000

class AutonomousFlight():
    def __init__(self):
        self.status = ""
        rospy.init_node('pid_bebop',anonymous=False)
        self.rate           = rospy.Rate(10)
        self.pubTakeoff     = rospy.Publisher("bebop/takeoff",Empty,queue_size=10)
        self.pubLand        = rospy.Publisher("bebop/land",Empty,queue_size=10)
        self.pubCommand     = rospy.Publisher("bebop/cmd_vel",Twist,queue_size=10)
        self.pubFlip        = rospy.Publisher("bebop/flip",UInt8,queue_size=10)
        
        #self.sub_data       = rospy.Subscriber("bebop/cmd_vel",Twist,self.data_callback) #callback test
        #rospy.Subscriber("bebop/cmd_vel",Twist,self.data_callback) #callback test - other way of callback
        self.subOdom        = rospy.Subscriber("bebop/odom",Odometry)

        self.setPoint       = np.array([0],[0],[0])
        self.position       = np.array([0],[0],[0])
        self.command        = Twist()
        self.state_change_time = rospy.Time.now()
        rospy.on_shutdown(self.send_land)
    
    #def data_callback(self, data):
    #    print(data)

    def error_callback(self, odom):
        print()
    
    def send_take_off(self):
        self.pubTakeoff.publish(Empty())
        print("takeoff")
        self.rate.sleep()

    def send_land(self):
        self.pubLand.publish(Empty())
        print("landing")

    def flip_me(self,data): # Only for Bebop; 0 - flip forward, 1 - flip backward, 2 - flip right, 3 - flip left
        self.pubFlip.publish(UInt8(data))
    
    def set_command(self,linear_x,linear_y,linear_z,angular_x,angular_y,angular_z):
        self.command.linear.x   = linear_x #forward (+) backward (-)
        self.command.linear.y   = linear_y #left (+) right (-)
        self.command.linear.z   = linear_z #up (+) down (-)
        self.command.angular.x  = angular_x
        self.command.angular.y  = angular_y #clockwise (-) anticlockwise (+)
        self.command.angular.z  = angular_z
        self.pubCommand.publish(self.command)
        self.rate.sleep()
    
    def stay(self, T):
        self.set_command(0,0,0,0,0,0)
        time.sleep(T)
    
    def forward(self,s):
        v = 1
        t = s / v
        for i in range(int(10 * t)):
            self.set_command(1,0,0,0,0,0)
            print("Go Forward: " + str(i))
    
    def Backward(self,s):
        v = 1
        t = s / v
        for i in range(int(10 * t)):
            self.set_command(-1,0,0,0,0,0)
            print("Go Backward: " + str(i))

    def go_left(self,s):
        v = 1
        t = s / v
        for i in range(int(10 * t)):
            self.set_command(0,1,0,0,0,0)
            print("Go Left: " + str(i))
    
    def go_right(self,s):
        v = 1
        t = s / v
        for i in range(int(10 * t)):
            self.set_command(0,-1,0,0,0,0)
            print("Go Right: " + str(i))
    
    def up(self,s):
        v = 1
        t = s / v
        for i in range(int(10 * t)):
            self.set_command(0,0,1,0,0,0)
            print("Go up: " + str(i))
    
    def down(self,s):
        v = 1
        t = s / v
        for i in range(int(10 * t)):
            self.set_command(0,0,-1,0,0,0)
            print("Go down: " + str(i))

    def turn_right(self,deg):
        rad = deg * (math.pi / 180)
        t = int(10*rad)
        for i in range(t):
            self.set_command(0,0,0,0,0,1)
        print("Turning " + str(deg) + " deg")
    
    def turn_left(self,deg):
        rad = deg * (math.pi / 180)
        t = int(10*rad)
        for i in range(t):
            self.set_command(0,0,0,0,0,-1)
        print("Turning " + str(deg) + " deg")
    
    def circle(self,r,t):
        circumference = 2 * math.pi * r
        v = circumference / t
        w = v / r
        for i in range(int(10 * t)):
            self.set_command(v,0,0,0,0,w)
        print("Circle")

if __name__ == '__main__':
    try:
        uav = AutonomousFlight()
        count = 0
        while not rospy.is_shutdown():
            i = 0
            #uav.send_take_off()
            #uav.stay(1)
            if count == 2:
                #uav.circle(0.5,10)
                #uav.stay(3)
                #uav.send_land()
                sys.exit()
            count+=1
    
    except rospy.ROSInterruptException:
        pass