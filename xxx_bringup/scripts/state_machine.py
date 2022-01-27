#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool, UInt8
def dummy_function(): pass
class Node :
    def __init__(self):
        rospy.init_node('state_machine')
        rospy.Subscriber('/button_map',Bool,self.callback_button_map)
        rospy.Subscriber('/button_save',Bool,self.callback_button_save)
        rospy.Subscriber('/button_cancel',Bool,self.callback_button_cancel)
        rospy.Subscriber('/button_start_nav',Bool,self.callback_button_nav)
        rospy.Subscriber('/button_finish_nav',Bool,self.callback_button_fin)
        
        self.pub_mode = rospy.Publisher('/mode',UInt8,queue_size=50)
        self.mode = 0
        self.dt = 0.1
        self.timer = rospy.Timer(rospy.Duration(self.dt), self.publish)
        self.process = None
    def callback_button_map(self,msg):
        if self.mode == 0:
            self.mode = 1
            # start mapping 
    def callback_button_save(self,msg):
        if self.mode == 1:
            self.mode = 0
            # shutdown mapping
            # run map saver
            print('save')
    def callback_button_cancel(self,msg):
        if self.mode == 1:
            self.mode = 0
            # shutdown mapping
            print('cancel')
    def callback_button_nav(self,msg):
        if self.mode == 0:
            self.mode = 2
            # start navigation
            
    def callback_button_fin(self,msg):
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