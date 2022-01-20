#!/usr/bin/env python3
import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Bool
from rospy.exceptions import ROSInterruptException
from turtlesim.msg import Pose as tPose
import numpy as np
import math
from std_srvs.srv import Empty, EmptyResponse

class Node :
    def __init__(self):
        rospy.init_node('turtlesim2pose')
        rospy.Subscriber('/turtle_pose',tPose,self.callback_turtle_pose)
        rospy.Subscriber('/turtle_goal',Pose,self.callback_turtle_goal)
        self.pub_current_pose = rospy.Publisher('current_pose',Pose,queue_size=10)
        self.pub_goal_pose = rospy.Publisher('goal_pose',Pose,queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.publish)
        self.current_pose = Pose()
        self.goal_pose = Pose()
    def callback_turtle_pose(self,msg):
        self.current_pose.position.x = msg.x
        self.current_pose.position.y = msg.y
        quat = quaternion_from_euler(0,0,msg.theta)
        self.current_pose.orientation.x = quat[0]
        self.current_pose.orientation.y = quat[1]
        self.current_pose.orientation.z = quat[2]
        self.current_pose.orientation.w = quat[3]
    def callback_turtle_goal(self,msg):
        self.goal_pose = msg
    def callback_shutDownTimer(self):
        self.timer.shutdown()
    def publish(self,event):
        self.pub_current_pose.publish(self.current_pose)
        self.pub_goal_pose.publish(self.goal_pose)

if __name__=='__main__':
    node = Node()
    rospy.on_shutdown(node.callback_shutDownTimer)
    rospy.spin()