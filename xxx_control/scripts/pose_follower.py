#!/usr/bin/env python3
from re import A
import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import Bool
from rospy.exceptions import ROSInterruptException
import numpy as np
import math
from std_srvs.srv import Empty, EmptyResponse

class Node :
    def __init__(self):
        rospy.init_node('pose_follower')
        service = rospy.Service('/enable_control',Empty,self.service_enable_control)
        service = rospy.Service('/disable_control',Empty,self.service_disable_control)
        
        rospy.Subscriber('/current_pose',Pose,self.callback_current_pose)
        rospy.Subscriber('/goal_pose',Pose,self.callback_goal_pose)
        self.pub_cmd = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
        self.pub_reach = rospy.Publisher('/has_reached',Bool,queue_size=10)
        
        self.timer = rospy.Timer(rospy.Duration(0.1), self.publish)
        self.has_reach = Bool()
        self.has_reach.data = False
        self.enable = True
        self.goal_2dpose = np.array([0,0,0])
        self.current_2dpose = np.array([0,0,0])
        self.cmd_vel = Twist()
    def pose2array(self,msg):
        x = msg.position.x
        y = msg.position.y
        quat = [msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w]
        euler_angles = euler_from_quaternion(quat)        
        theta = euler_angles[2]
        return [x,y,theta]
    def callback_current_pose(self,msg):
        self.current_2dpose  = self.pose2array(msg)
    def callback_goal_pose(self,msg):
        self.goal_2dpose  = self.pose2array(msg)
    def callback_shutDownTimer(self):
        self.cmd_vel.linear.x = 0
        self.cmd_vel.angular.z = 0
        self.pub_cmd.publish(self.cmd_vel)
        self.timer.shutdown()
    def actual_control(self):
        Kp = 1.3
        Kb = 1.5
        Ka = Kp*5
        dp = np.array([self.goal_2dpose[0],self.goal_2dpose[1]])-np.array([self.current_2dpose[0],self.current_2dpose[1]])
        p = np.linalg.norm(dp)
        a = math.atan2(dp[1],dp[0])-self.current_2dpose[2]
        ap = math.atan2(math.sin(a),math.cos(a))
        b = -self.current_2dpose[2]-ap
        if p<0.02 :
            v = 0
            w = -Kb*(self.current_2dpose[2]-self.goal_2dpose[2])
            self.has_reach.data = True
        else:
            self.has_reach.data = False
            v = Kp*p
            w = Ka*ap+Kb*(math.atan2(dp[1],dp[0])-self.goal_2dpose[2])
        return v,w
    def publish(self,event):
        if self.enable:
            v,w = self.actual_control() #control()
            self.cmd_vel.linear.x = v
            self.cmd_vel.angular.z = w
            self.pub_cmd.publish(self.cmd_vel)
            self.pub_reach.publish(self.has_reach)
        else:
            self.has_reach = False
            self.pub_reach.publish(self.has_reach)
    def service_enable_control(self,request):
        self.enable = True
        response =  EmptyResponse()
        return response
    def service_disable_control(self,request):
        self.enable = False
        response =  EmptyResponse()
        return response

if __name__=='__main__':
    node = Node()
    rospy.on_shutdown(node.callback_shutDownTimer)
    rospy.spin()
