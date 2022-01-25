#!/usr/bin/env python3
import math
from math import sin, cos, pi
import numpy as np
import rospy
from std_msgs.msg import UInt8, Float64
class Node :
    def __init__(self):
        rospy.init_node('sim_encoder')
        self.pub_enc = rospy.Publisher('/encoder',UInt8,queue_size=50)
        self.pub_vel = rospy.Publisher('/ground_truth',Float64,queue_size=50)
        
        self.init_time = rospy.Time.now()
        self.timer = rospy.Timer(rospy.Duration(0.05), self.publish)
    def callback_shutDownTimer(self):
        self.timer.shutdown()
    def publish(self,event):
        msg_enc = UInt8()
        msg_vel = Float64()
        f = 0.1
        R = 255*1.5
        dt = rospy.Time.now().secs+1e-09*rospy.Time.now().nsecs-self.init_time.secs-1e-09*self.init_time.nsecs
        val = R*sin(2*pi*f*dt)+20*(np.random.rand(1,1)-0.5)
        msg_enc.data = np.uint8(val[0,0] % 255)
        msg_vel.data = 2*pi*f*R*cos(2*pi*f*dt)
        # publish the message
        self.pub_enc.publish(msg_enc)
        self.pub_vel.publish(msg_vel)

if __name__=='__main__':
    node = Node()
    rospy.on_shutdown(node.callback_shutDownTimer)
    rospy.spin()