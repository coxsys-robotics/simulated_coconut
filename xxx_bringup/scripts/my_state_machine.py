#!/usr/bin/env python3
import rospy 
from std_msgs.msg import Bool, UInt8
class Node :
    def __init__(self):
        rospy.init_node('state_machine')
        rospy.Subscriber('/button_map',Bool,self.callback_button_map)
        rospy.Subscriber('/button_sav',Bool,self.callback_button_sav)
        rospy.Subscriber('/button_nav',Bool,self.callback_button_nav)
        rospy.Subscriber('/button_can',Bool,self.callback_button_can)
        rospy.Subscriber('/button_fin',Bool,self.callback_button_fin)
        
        self.pub_mode = rospy.Publisher('/mode',UInt8,queue_size=50)
        self.mode = 0
    def pubMode(self):
        mode_msg = UInt8()
        mode_msg.data = self.mode
        self.pub_mode.publish(mode_msg)
    def callback_button_map(self,msg):
        if self.mode == 0:
            self.mode = 1
            self.pubMode()
    def callback_button_sav(self,msg):
        if self.mode == 1:
            print('save')
            self.mode = 0
            self.pubMode()
    def callback_button_can(self,msg):
        if self.mode == 1:
            print('cancel')
            self.mode = 0
            self.pubMode()
    def callback_button_nav(self,msg):
        if self.mode == 0:
            self.mode = 2
            self.pubMode()
    def callback_button_fin(self,msg):
        if self.mode == 2:
            print('navigation is complete')
            self.mode = 0
            self.pubMode()

if __name__=='__main__':
    node = Node()
    rospy.spin()