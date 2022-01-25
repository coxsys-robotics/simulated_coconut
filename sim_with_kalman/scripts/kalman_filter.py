#!/usr/bin/env python3
import math
from math import sin, cos, pi
import numpy as np
import rospy
from std_msgs.msg import Float64
class Node :
    def __init__(self):
        rospy.init_node('kalman_filter')
        rospy.Subscriber('/encoder',Float64,self.callback_encoder)
        
        self.pub_vel = rospy.Publisher('/enc_vel',Float64,queue_size=50)
        self.pub_var = rospy.Publisher('/var_vel',Float64,queue_size=50)
        
        self.enc = 0
        self.x_est = 
        self.P_est = 
        self.dt = 0.1
        self.timer = rospy.Timer(rospy.Duration(0.1), self.publish)
    def callback_encoder(self,msg):
        self.enc = msg.data
    def callback_shutDownTimer(self):
        self.timer.shutdown()
    def publish(self,event):
        msg_vel = Float64()
        msg_var = Float64()
        
        F = 
        B = 
        u = 
        G = 
        C = 
        Q = 
        R = 
        x_predict = np.matmul(F,self.x_est)+np.matmul(B,u)
        P_predict = np.matmul(F,np.matmul(self.P_est,np.transpose(F)))+np.matmul(G,np.matmul(Q,np.transpose(G)))

        y_res = self.enc-np.matmul(C,x_predict)
        S = np.matmul(C,np.matmul(P_predict,np.transpose(C)))+R
        K = np.matmul(P_predict,np.matmul(np.transpose(C),np.linalg.inv(S)))
        self.x_est = x_predict+np.matmul(K,y_res)
        self.P_est = P_predict-np.matmul(K,np.matmul(C,P_predict))

        msg_vel.data = self.x_est[1]
        msg_var.data = self.P_est[1,1]
        
        # publish the message
        self.pub_vel.publish(msg_vel)
        self.pub_var.publish(msg_var)

if __name__=='__main__':
    node = Node()
    rospy.on_shutdown(node.callback_shutDownTimer)
    rospy.spin()