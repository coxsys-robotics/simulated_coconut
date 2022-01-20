#!/usr/bin/env python3
import math
from math import sin, cos, pi
import numpy as np
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty, EmptyResponse
class Node :
    def __init__(self):
        rospy.init_node('twist_to_wheel_velocity')
        rospy.Subscriber('cmd_vel',Twist,self.callback_cmd_vel)
        self.pub_wheel_right = rospy.Publisher('cmd_right',Float64,queue_size=50)
        self.pub_wheel_left = rospy.Publisher('cmd_left',Float64,queue_size=50)
        self.cmd_vel = Twist()
        #self.dt = 0.1
        #self.timer = rospy.Timer(rospy.Duration(self.dt), self.publish)
    def callback_cmd_vel(self,msg):
        self.cmd_vel = msg
        self.publish()
    #def callback_shutDownTimer(self):
    #    self.timer.shutdown()
    def publish(self):
        r = 0.08
        b = 0.45
        v = self.cmd_vel.linear.x
        w = self.cmd_vel.angular.z
        speed_left = Float64()
        speed_left.data = (v-b/2*w)/r
        speed_right = Float64()
        speed_right.data = (v+b/2*w)/r
        
        # publish the message
        self.pub_wheel_right.publish(speed_right)
        self.pub_wheel_left.publish(speed_left)
    
if __name__=='__main__':
    node = Node()
    #rospy.on_shutdown(node.callback_shutDownTimer)
    rospy.spin()