#!/usr/bin/env python3
import os, pathlib
import math
import numpy as np
import rospy
from sensor_msgs.msg import Imu
import rosbag

class Node:
    def __init__(self,node_name):
        self.node_name = node_name
        rospy.init_node(self.node_name)

        if rospy.has_param('~sub_topic'):
            self.topic_imu = rospy.get_param('~sub_topic')
        else:
            rospy.set_param('~sub_topic','imu')
            print('using default sub_topic as ''imu'' ')
            self.topic_imu = 'imu'
        
        if rospy.has_param('~num_sample'):
            self.num_sample = rospy.get_param('~num_sample')
        else:
            rospy.set_param('~num_sample',100)
            print('using default number of sample as 100 ')
            self.num_sample = 100
        
        if rospy.has_param('~num_experiment'):
            self.num_experiment = rospy.get_param('~num_experiment')
        else:
            rospy.set_param('~num_experiment',5)
            print('using default number of experiment as 5 ')
            self.num_experiment = 5

        rospy.Subscriber(self.topic_imu,Imu,self.callback_imu)
        self.imu_msg = Imu()
        self.bag_idx = 0
        self.sample_idx = 0
        self.finish = True
        self.log = False
        self.logging = False
        self.calibrator = None
        self.bag = None
        rospy.on_shutdown(self.callback_shutdown) 
    def callback_imu(self,msg):
        self.imu_msg = msg
        if not self.logging:
            if self.log:
                self.logging = True
                self.finish = True
                self.bag_idx = 0
        elif self.logging:
            if self.finish:
                self.bag_idx += 1
                if not (self.bag is None):
                    self.bag.close()
                if self.bag_idx > self.num_experiment:
                    self.logging = False
                    print('Finish logging')
                    self.log = False
                if self.log:
                    path = pathlib.Path(__file__).parent.resolve()
                    path = os.path.dirname(path)
                    self.bag = rosbag.Bag(path+'/data/'+'imu'+str(self.bag_idx)+'.bag', 'w')
                    self.finish = False 
                    self.sample_idx = 0
            else:
                self.bag.write(self.topic_imu,msg)
                self.sample_idx += 1
                if self.sample_idx>=self.num_sample:
                    self.finish = True
                    print('Bag'+str(self.bag_idx)+' is logged.')
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