#!/usr/bin/env python3
from operator import matmul
import rospy
import tf
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix, quaternion_from_matrix
from geometry_msgs.msg import Pose, Twist, Transform, TransformStamped
from std_msgs.msg import Bool
from rospy.exceptions import ROSInterruptException
import numpy as np
import math
from std_srvs.srv import Empty, EmptyResponse

class Node :
    def __init__(self):
        rospy.init_node('aruco2robot')
        rospy.Subscriber('/gazebo/model_states',ModelStates,self.callback_model_states)
        self.pose = Pose()
        self.pub_current_pose = rospy.Publisher('/current_pose',Pose,queue_size=10)
        self.pub_goal_pose = rospy.Publisher('/goal_pose',Pose,queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.publish)
        
    def callback_model_states(self,msg):
        
        if len(msg.pose)>=6:
            id = 0
            for model in msg.name:
                if model=='xxx_robot':
                    self.pose = msg.pose[id]
                    print(msg.name[id])
                id = id + 1
    def publish(self,event):
        d = 0.75
        H_0_G = np.array([[1,0,0,-d],[0,1,0,4],[0,0,1,0],[0,0,0,1]])
        H_G_0 = np.linalg.inv(H_0_G)
        p = np.array([self.pose.position.x,self.pose.position.y,self.pose.position.z])
        q = np.array([self.pose.orientation.x,self.pose.orientation.y,self.pose.orientation.z,self.pose.orientation.w])
        H_0_R = quaternion_matrix(q)
        H_0_R[0:3,-1] = p
        H = np.matmul(H_G_0,H_0_R)
        quat_r = quaternion_from_matrix(H)
        current_pose_msg = Pose()
        current_pose_msg.position.x = H[0][3]
        current_pose_msg.position.y = H[1][3]
        current_pose_msg.orientation.x = quat_r[0]
        current_pose_msg.orientation.y = quat_r[1]
        current_pose_msg.orientation.z = quat_r[2]
        current_pose_msg.orientation.w = quat_r[3]
            
        goal_pose_msg = Pose()
        self.pub_current_pose.publish(current_pose_msg)
        self.pub_goal_pose.publish(goal_pose_msg)
    def callback_shutDownTimer(self):
        self.timer.shutdown()
if __name__=='__main__':
    node = Node()
    rospy.on_shutdown(node.callback_shutDownTimer)
    rospy.spin()
