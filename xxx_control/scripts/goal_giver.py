#!/usr/bin/env python3
import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Bool
from rospy.exceptions import ROSInterruptException
import numpy as np
import math
from std_srvs.srv import Empty, EmptyResponse

class Node :
    def __init__(self):
        rospy.init_node('goal_giver')
        service = rospy.Service('/rand_goal',Empty,self.service_rand_goal)
        self.pub_goal_pose = rospy.Publisher('/goal',Pose,queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.publish)
        self.goal_pose = Pose()
        goal_p = 10*np.random.rand(2)
        self.goal_theta = math.pi/4 #2*math.pi*np.random.rand(1)
        self.goal_pose.position.x = 2.5 #goal_p[0]
        self.goal_pose.position.y = 7.5 #goal_p[1]
        quat = quaternion_from_euler(0,0,self.goal_theta)
        self.goal_pose.orientation.x = quat[0]
        self.goal_pose.orientation.y = quat[1]
        self.goal_pose.orientation.z = quat[2]
        self.goal_pose.orientation.w = quat[3]
    def callback_shutDownTimer(self):
        self.timer.shutdown()
    def service_rand_goal(self,request):
        goal_p = 10*np.random.rand(2)
        self.goal_theta = 2*math.pi*np.random.rand(1)
        self.goal_pose.position.x = goal_p[0]
        self.goal_pose.position.y = goal_p[1]
        quat = quaternion_from_euler(0,0,self.goal_theta)
        self.goal_pose.orientation.x = quat[0]
        self.goal_pose.orientation.y = quat[1]
        self.goal_pose.orientation.z = quat[2]
        self.goal_pose.orientation.w = quat[3]
        response =  EmptyResponse()
        return response
    def publish(self,event):
        self.pub_goal_pose.publish(self.goal_pose)
if __name__=='__main__':
    node = Node()
    rospy.on_shutdown(node.callback_shutDownTimer)
    rospy.spin()

    