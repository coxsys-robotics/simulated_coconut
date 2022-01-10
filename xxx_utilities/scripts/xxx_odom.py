#!/usr/bin/env python3
import math
from math import sin, cos, pi
import numpy as np
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty, EmptyResponse
class Node :
    def __init__(self):
        rospy.init_node('wheel_vel_to_odom')
        srv_set_odom = rospy.Service('/set_odom',Empty,self.service_set_odom)
        rospy.Subscriber('/joint_states',JointState,self.callback_joint_state)
        rospy.Subscriber('/pose_to_set',Pose,self.callback_pose_to_set)
        self.pub_odom = rospy.Publisher('/odom',Odometry,queue_size=50)
        self.odom_broadcaster = tf.TransformBroadcaster()
        self.wheel = np.array([0,0])
        self.x = 0
        self.y = 0
        self.th = 0
        self.vx = 0
        self.vy = 0
        self.vth = 0
        self.dt = 0.1
        self.pose_to_set = Pose()
        self.timer = rospy.Timer(rospy.Duration(self.dt), self.publish)
    def callback_joint_state(self,msg):
        self.wheel = np.array(msg.velocity)
    def callback_pose_to_set(self,msg):
        self.pose_to_set = msg
    def callback_shutDownTimer(self):
        self.timer.shutdown()
    def publish(self,event):
        r = 0.08
        b = 0.45
        self.integrate(r,b)
        current_time = rospy.Time.now()
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)
        self.odom_broadcaster.sendTransform((self.x, self.y, 0.),odom_quat,current_time,"base_footprint","odom")
        
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"

        # set the position
        odom.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*odom_quat))

        # set the velocity
        odom.child_frame_id = "base_footprint"
        odom.twist.twist = Twist(Vector3(self.vx, self.vy, 0), Vector3(0, 0, self.vth))

        # publish the message
        self.pub_odom .publish(odom)
    def service_set_odom(self,request):
        self.x = self.pose_to_set.position.x
        self.y = self.pose_to_set.position.y
        q = [self.pose_to_set.orientation.x,self.pose_to_set.orientation.y,self.pose_to_set.orientation.z,self.pose_to_set.orientation.w]
        (rx,ry,rz)=tf.transformations.quaternion_from_euler()
        self.th = rz
        response =  EmptyResponse()
        return response
    def integrate(self,r,b):
        self.vx = (self.wheel[0]+self.wheel[1])*r/2
        self.vy = 0
        self.vth = (self.wheel[0]-self.wheel[1])*r/b
        delta_x = (self.vx * cos(self.th) - self.vy * sin(self.th)) * self.dt
        delta_y = (self.vx * sin(self.th) + self.vy * cos(self.th)) * self.dt
        delta_th = self.vth * self.dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

if __name__=='__main__':
    node = Node()
    rospy.on_shutdown(node.callback_shutDownTimer)
    rospy.spin()