#!/usr/bin/env python3
import math
from math import sin, cos, pi
import numpy as np
import rospy
import tf
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty, EmptyResponse
from std_msgs.msg import Float32
class JointStateVel :
    def __init__(self):
        rospy.init_node('joint_state_vel')
        self.sub = rospy.Subscriber('/xxx/joint_states',JointState, self.callback_joint_state_vel)
        self.veldata = JointState()
        ##
        self.pub_joint_vel = rospy.Publisher('/xxx/joint_velocity',Float32,queue_size=50)
        dt = 0.01
        self.timer = rospy.Timer(rospy.Duration(dt), self.publish_joint_state_vel)

    def callback_joint_state_vel(self,msg):
        self.veldata = msg.velocity
        # print(self.veldata)
  
    def callback_shutDownTimer(self):
        self.timer.shutdown()
    ###
    def publish_joint_state_vel(self,event):
        joint_vel = Float32()
        joint_vel.data = self.veldata[0]
        # joint_vel.data = 0.001
        self.pub_joint_vel.publish(joint_vel)

if __name__=='__main__':
    joint_state_vel = JointStateVel()
    rospy.on_shutdown(joint_state_vel.callback_shutDownTimer)
    rospy.spin()