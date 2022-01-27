#!/usr/bin/env python3
import rospy
import tf
from geometry_msgs.msg import Pose, Twist, Transform, TransformStamped, PoseWithCovarianceStamped
import numpy as np
import math

class Node :
    def __init__(self):
        rospy.init_node('warp_prevention')
        rospy.Subscriber('/amcl_pose',PoseWithCovarianceStamped,self.callback_amcl_pose)
        self.pub_initial_pose = rospy.Publisher('/initial_pose',PoseWithCovarianceStamped,queue_size=10)
        self.pose = PoseWithCovarianceStamped()
    def callback_amcl_pose(self,msg):
        hasWarped = False
        if hasWarped:
            self.publish()
        else:
            self.pose = msg
    def publish(self):
        initial_pose_msg = PoseWithCovarianceStamped()
        initial_pose_msg.pose = self.pose
        self.pub_initial_pose.publish(initial_pose_msg)
    def callback_shutDownTimer(self):
        self.timer.shutdown()
if __name__=='__main__':
    node = Node()
    rospy.on_shutdown(node.callback_shutDownTimer)
    rospy.spin()
