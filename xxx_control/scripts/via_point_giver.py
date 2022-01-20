#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from rospy.exceptions import ROSInterruptException
from turtlesim.msg import Pose as tPose
import numpy as np
import math
from std_srvs.srv import Empty, EmptyResponse

class Node :
    def __init__(self):
        self.via_points = np.array([[2,4,6,1,7],[2,6,2,4,4]])
        self.isAtEnd = False
        self.idx = 0
        self.pub_msg = tPose()
        self.timer = None
    def callback_isThere(self,msg):
        if msg.data and (self.idx+1<self.via_points.shape[1]):
            self.idx += 1
            self.isAtEnd = False
        if (self.idx+1)==self.via_points.shape[1]:
            self.isAtEnd = True
        isAtEnd_msg = Bool()
        isAtEnd_msg.data = self.isAtEnd
        pub_isAtEnd.publish(isAtEnd_msg)
    def callback_shutDownTimer(self):
        self.timer.shutdown()
    def service_reset_goal(self,request):
        self.idx = 0
        self.isAtEnd = False
        isAtEnd_msg = Bool()
        isAtEnd_msg.data = False
        pub_isAtEnd.publish(isAtEnd_msg)
        response =  EmptyResponse()
        return response
    def publish(self,event):
        goal = self.via_points[:,self.idx]
        self.pub_msg.x = goal[0]
        self.pub_msg.y = goal[1]
        pub_goal_pose.publish(self.pub_msg)
        isAtEnd_msg = Bool()
        isAtEnd_msg.data = self.isAtEnd
        pub_isAtEnd.publish(isAtEnd_msg)
if __name__=='__main__':
    rospy.init_node('command_giver')
    node = Node()
    rospy.Subscriber('/isThere',Bool,node.callback_isThere)
    service = rospy.Service('/reset_goal',Empty,node.service_reset_goal)
    pub_goal_pose = rospy.Publisher('/goal_pose',tPose,queue_size=10)
    pub_isAtEnd = rospy.Publisher('/isAtEnd',Bool,queue_size=10)
    node.timer = rospy.Timer(rospy.Duration(0.1), node.publish)
    rospy.on_shutdown(node.callback_shutDownTimer)
    rospy.spin()

    