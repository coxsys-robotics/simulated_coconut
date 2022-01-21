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
class JointStateEffort :
    def __init__(self):
        rospy.init_node('joint_state_effort')
        self.sub = rospy.Subscriber('/xxx/joint_states',JointState, self.callback_joint_state_effort)
        self.effortdata = JointState()
        ##
        self.pub_joint_effort = rospy.Publisher('/xxx/joint_effort',Float32,queue_size=50)
        dt = 0.01
        self.timer = rospy.Timer(rospy.Duration(dt), self.publish_joint_state_effort)

    def callback_joint_state_effort(self,msg):
        self.effortdata = msg.effort
        # print(self.effortdata[0])
  
    def callback_shutDownTimer(self):
        self.timer.shutdown()
    ###
    def publish_joint_state_effort(self,event):
        joint_effort = Float32()
        joint_effort.data = self.effortdata[0]
        # joint_effort.data = 0.001
        self.pub_joint_effort.publish(joint_effort)

if __name__=='__main__':
    joint_state_effort = JointStateEffort()
    rospy.on_shutdown(joint_state_effort.callback_shutDownTimer)
    rospy.spin()