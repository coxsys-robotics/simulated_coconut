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
            H =
            quat =
            current_pose_msg = Pose()
            current_pose_msg.position.x = H[0][3]
            current_pose_msg.position.y = H[1][3]
            current_pose_msg.orientation.x = quat[0]
            current_pose_msg.orientation.y = quat[1]
            current_pose_msg.orientation.z = quat[2]
            current_pose_msg.orientation.w = quat[3]
            goal_pose_msg = Pose()
            self.pub_current_pose.publish(current_pose_msg)
            self.pub_goal_pose.publish(goal_pose_msg)
    def callback_shutDownTimer(self):
        self.timer.shutdown()
if __name__=='__main__':
    node = Node()
    rospy.on_shutdown(node.callback_shutDownTimer)
    rospy.spin()
