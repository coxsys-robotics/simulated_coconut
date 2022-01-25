#!/usr/bin/env python3
import math
from math import sin, cos, pi
import numpy as np
import rospy
from std_msgs.msg import UInt8, Float64
class Node :
    def __init__(self):
        rospy.init_node('unwrapper')
        rospy.Subscriber('/encoder',UInt8,self.callback_encoder)
        self.pub_enc = rospy.Publisher('/unwrap_enc',Float64,queue_size=50)
        
        self.enc = 0
        self.offset = 0
        self.previous = 0
        self.dt = 0.1
        self.timer = rospy.Timer(rospy.Duration(self.dt), self.publish)
    def callback_encoder(self,msg):
        self.previous = self.enc
        self.enc = msg.data
        if self.enc-self.previous>255*0.75:
            self.offset = self.offset-255
        if self.enc-self.previous<-255*0.75:
            self.offset = self.offset+255
    def callback_shutDownTimer(self):
        self.timer.shutdown()
    def publish(self,event):
        msg_enc = Float64()
        msg_enc.data = self.enc+self.offset
        # publish the message
        self.pub_enc.publish(msg_enc)


if __name__=='__main__':
    node = Node()
    rospy.on_shutdown(node.callback_shutDownTimer)
    rospy.spin()