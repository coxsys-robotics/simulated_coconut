#!/usr/bin/env python3
import yaml
import sys,os, glob, pathlib
import numpy as np
import rospy
import rosbag

class Node:
    def __init__(self,node_name):
        self.node_name = node_name
        rospy.init_node(self.node_name)
        if len(sys.argv)>1 and rospy.has_param('~sub_topic'):
            self.topic_imu = rospy.get_param('~sub_topic')
        else:
            rospy.set_param('~sub_topic','imu')
            print('using default sub_topic as ''imu'' ')
            self.topic_imu = 'imu'
        
        rospy.on_shutdown(self.callback_shutdown)
    def callback_shutdown(self):
        print('\n\n... The node "'+self.node_name+'" has shut down...\n')
class Calibrator:
    def __init__(self,sample_size,num_exp) -> None:
        self.sample_size = sample_size
        self.num_exp = num_exp
        
    def load_data(self,bag_file,exp_idx,topic_name):
        pass
# main
if __name__=="__main__":
    
    node = Node('imu_calibration')
    
    print('Covariance Matrices have been added to imu_config.yaml.')
    print('You can shut this node down.')

    rospy.spin()