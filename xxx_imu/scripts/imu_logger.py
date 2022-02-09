#!/usr/bin/env python3
import os, pathlib
import rospy
from sensor_msgs.msg import Imu
import rosbag

class Node:
    def __init__(self,node_name):
        self.node_name = node_name
        rospy.init_node(self.node_name)

        rospy.Subscriber(self.topic_imu,Imu,self.callback_imu)
        self.imu_msg = Imu()
        
        self.log = False
        
        rospy.on_shutdown(self.callback_shutdown) 
    def callback_imu(self,msg):
        self.imu_msg = msg
        if not self.logging:
            if self.log:
                pass
                # do something
        elif self.logging:
            pass
            # do something
    def log_data(self):
        self.log = True
    def callback_shutdown(self):
        print('\n\n... The node "'+self.node_name+'" has shut down...\n')

if __name__=="__main__":
    
    node = Node('imu_calibration')
    node.log_data()
    while node.log:
        pass  # logging
    
    print('IMU data have been logged to xxx_imu/data.')
    print('You can shut this node down.')

    rospy.spin()