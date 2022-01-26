#!/usr/bin/env python3
import pygame
import rospy 
from std_msgs.msg import Bool, UInt8

class Node :
    def __init__(self):
        rospy.init_node('state_machine')
        rospy.Subscriber('/mode_map',Bool,self.callback_mode_map)
        rospy.Subscriber('/mode_save',Bool,self.callback_mode_save)
        rospy.Subscriber('/mode_cancel',Bool,self.callback_mode_cancel)
        rospy.Subscriber('/mode_start_nav',Bool,self.callback_mode_nav)
        rospy.Subscriber('/mode_finish_nav',Bool,self.callback_mode_fin)
        self.pub_mode = rospy.Publisher('/mode',UInt8,queue_size=50)
        self.mode = 0
        self.dt = 0.1
        self.timer = rospy.Timer(rospy.Duration(self.dt), self.publish)
    def callback_mode_map(self,msg):
        if self.mode == 0:
            self.mode = 1
            # start mapping 
    def callback_mode_save(self,msg):
        if self.mode == 1:
            self.mode = 0
            # shutdown mapping
            # run map saver
            print('save')
    def callback_mode_cancel(self,msg):
        if self.mode == 1:
            self.mode = 0
            # shutdown mapping
            print('cancel')
    def callback_mode_nav(self,msg):
        if self.mode == 0:
            self.mode = 2
            # start navigation
    def callback_mode_fin(self,msg):
        if self.mode == 2:
            self.mode = 0
            # shutdown navigation
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