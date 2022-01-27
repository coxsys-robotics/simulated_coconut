#!/usr/bin/env python3
import pygame
import rospy 
from std_msgs.msg import Bool, UInt8

class Node :
    def __init__(self):
        rospy.init_node('state_machine')
        rospy.Subscriber('/button_map',Bool,self.callback_button_map)
        self.pub_mode = rospy.Publisher('/mode',UInt8,queue_size=50)
        self.mode = 0
        self.dt = 0.1
        self.timer = rospy.Timer(rospy.Duration(self.dt), self.publish)
    def callback_button_map(self,msg):
        print('update')
    def callback_shutDownTimer(self):
        self.timer.shutdown()
    def publish(self,event):
        mode_msg = UInt8()
        mode_msg.data = self.mode
        self.pub_mode.publish(mode_msg)

if __name__=='__main__':
    node = Node()
    rospy.on_shutdown(node.callback_shutDownTimer)
    rospy.spin()