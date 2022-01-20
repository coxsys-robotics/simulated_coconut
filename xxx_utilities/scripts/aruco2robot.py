#!/usr/bin/env python3
from operator import matmul
import rospy
import tf
from fiducial_msgs.msg import FiducialTransformArray, FiducialTransform
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
        rospy.Subscriber('/fiducial_transforms',FiducialTransformArray,self.callback_fiducial_transforms)
        self.id = 0
        self.pub_current_pose = rospy.Publisher('/current_pose',Pose,queue_size=10)
        self.pub_goal_pose = rospy.Publisher('/goal_pose',Pose,queue_size=10)
        self.H = Transform()
        self.timer = rospy.Timer(rospy.Duration(0.1), self.publish)
        self.fiducial_tf = FiducialTransformArray()
    def callback_fiducial_transforms(self,msg):
        self.fiducial_tf = msg
    def publish(self,event):
        H_id = Transform()
        flag = False
        for H in self.fiducial_tf.transforms:
            if H.fiducial_id == self.id:
                H_id = H.transform
                flag = True
        if flag:
            p = np.array([H_id.translation.x,H_id.translation.y,H_id.translation.z])
            q = np.array([H_id.rotation.x,H_id.rotation.y,H_id.rotation.z,H_id.rotation.w])
            H_C_A = quaternion_matrix(q)
            H_C_A[0:3,-1] = p
            H_A_C = np.linalg.inv(H_C_A)
            h = 0.2
            d = 0.75
            cd = 0.317
            ch = 0.265
            H_G_A = np.array([[0,0,-1,d],[-1,0,0,0],[0,1,0,h],[0,0,0,1]])
            H_C_R = np.array([[0,-1,0,0],[0,0,-1,ch],[1,0,0,-cd],[0,0,0,1]])
            H = np.matmul(H_G_A,np.matmul(H_A_C,H_C_R))
            quat_r = quaternion_from_matrix(H)
            #quat = [H_id.rotation.x,H_id.rotation.y,H_id.rotation.z,H_id.rotation.w]
            #p = np.array([H_id.translation.x,H_id.translation.y,H_id.translation.z])
            #R_bi = np.array(quaternion_matrix(quat))
            #R = np.linalg.inv(R_bi[0:-1,0:-1])

            #p_a = np.matmul(R,-p)
            #R_G_A = np.array([[0,0,1],[1,0,0],[0,0,1]])
            #R_A_C = R
            #R_C_R = np.array([[0,-1,0],[0,0,-1],[1,0,0]])
            #R_G_R = np.matmul(R_G_A,np.matmul(R_A_C,R_C_R))
            #H = np.concatenate((R_G_R,np.array([[0,0,0]]).transpose()),axis=1)
            #H = np.concatenate((H,np.array([[0,0,0,1]])),axis=0)
            #quat_r = quaternion_from_matrix(H)
            
            current_pose_msg = Pose()
            current_pose_msg.position.x = H[0][3]
            current_pose_msg.position.y = H[1][3]
            current_pose_msg.orientation.x = quat_r[0]
            current_pose_msg.orientation.y = quat_r[1]
            current_pose_msg.orientation.z = quat_r[2]
            current_pose_msg.orientation.w = quat_r[3]
            goal_pose_msg = Pose()
            #d = 0.6
            #goal_pose_msg.position.x = 0
            #quat_g = quaternion_from_euler(0,0,0)
            #goal_pose_msg.orientation.x = quat_g[0]
            #goal_pose_msg.orientation.y = quat_g[1]
            #goal_pose_msg.orientation.z = quat_g[2]
            #goal_pose_msg.orientation.w = quat_g[3]
            self.pub_current_pose.publish(current_pose_msg)
            self.pub_goal_pose.publish(goal_pose_msg)
    def callback_shutDownTimer(self):
        self.timer.shutdown()
if __name__=='__main__':
    node = Node()
    rospy.on_shutdown(node.callback_shutDownTimer)
    rospy.spin()
