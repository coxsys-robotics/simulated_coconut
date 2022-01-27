#!/usr/bin/env python3
import rospy
from actionlib_msgs.msg import GoalStatusArray

class Node :
    def __init__(self):
        rospy.init_node('check_status')
        rospy.Subscriber("/move_base/status", GoalStatusArray, self.callback_status)
    def callback_status(self,msg):
        print(msg.status_list[0].status)
if __name__=='__main__':
    node = Node()
    rospy.spin()
