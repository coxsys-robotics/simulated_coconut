#!/usr/bin/env python3
import rospy
import tf2_ros
import geometry_msgs.msg
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
class Node:
    def __init__(self,node_name):
        self.node_name = node_name
        rospy.init_node(self.node_name)

        rospy.Subscriber('imu/data',Imu,self.callback_imu)
        self.imu_msg = Imu()
        rospy.on_shutdown(self.callback_shutdown) 
    def callback_imu(self,msg):

        self.imu_msg = msg
        br = tf2_ros.TransformBroadcaster()
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'imu'
        t.transform.rotation.x = msg.orientation.x
        t.transform.rotation.y = msg.orientation.y
        t.transform.rotation.z = msg.orientation.z
        t.transform.rotation.w = msg.orientation.w

        br.sendTransform(t)
    def callback_shutdown(self):
        print('\n\n... The node "'+self.node_name+'" has shut down...\n')
# main
if __name__=="__main__":
    
    node = Node('tf_imu')
    rospy.spin()