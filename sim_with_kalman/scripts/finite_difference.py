#!/usr/bin/env python3
import math
from math import sin, cos, pi
import numpy as np
import rospy
from std_msgs.msg import Float64
class Node :
    def __init__(self):
        rospy.init_node('finite_differene')
        rospy.Subscriber('/encoder',Float64,self.callback_encoder)
        self.pub_vel = rospy.Publisher('/enc_vel',Float64,queue_size=50)
        self.enc = 0
        
        self.init = True
        self.dt = 0.1
        self.timer = rospy.Timer(rospy.Duration(self.dt), self.publish)
    def callback_encoder(self,msg):

        self.enc = msg.data
    def callback_shutDownTimer(self):
        self.timer.shutdown()
    def publish(self,event):
        if self.init:
            self.init = False
        else:
            msg_vel = Float64()
            msg_vel.data = 
            self.pub_vel.publish(msg_vel)

if __name__=='__main__':
    node = Node()
    rospy.on_shutdown(node.callback_shutDownTimer)
    rospy.spin()