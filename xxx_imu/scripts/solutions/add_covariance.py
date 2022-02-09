#!/usr/bin/env python3
import yaml
import os, glob, pathlib
import rospy
from sensor_msgs.msg import Imu

class Node:
    def __init__(self,node_name):
        self.node_name = node_name
        rospy.init_node(self.node_name)

        if rospy.has_param('~sub_topic'):
            self.topic_sub = rospy.get_param('~sub_topic')
        else:
            rospy.set_param('~sub_topic','imu')
            print('using default sub_topic as ''imu'' ')
            self.topic_sub = 'imu'
        
        if rospy.has_param('~pub_topic'):
            self.topic_pub = rospy.get_param('~pub_topic')
        else:
            rospy.set_param('~pub_topic','filtered_imu')
            print('using default pub_topic as ''filtered_imu'' ')
            self.topic_pub = 'filtered_imu'
        

        rospy.Subscriber(self.topic_sub,Imu,self.callback_imu)
        self.pub_imu = rospy.Publisher(self.topic_pub,Imu,queue_size=10)

        self.imu_msg = Imu()
        path = pathlib.Path(__file__).parent.resolve()
        path = os.path.dirname(path)  
        file = glob.glob(path+'/config/imu_config.yaml')[0]
    
        with open(file, "r") as stream:
            try:
                info = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                print(exc)
        self.cov_w = info['angular_velocity_covariance']
        self.cov_a = info['linear_acceleration_covariance']
        
        rospy.on_shutdown(self.callback_shutdown) 
    def callback_imu(self,msg):
        self.imu_msg = msg
        self.imu_msg.angular_velocity_covariance = self.cov_w
        self.imu_msg.linear_acceleration_covariance = self.cov_a
        
        self.pub_imu.publish(self.imu_msg)
    def callback_shutdown(self):
        print('\n\n... The node "'+self.node_name+'" has shut down...\n')
# main
if __name__=="__main__":
    
    node = Node('imu_with_covariance')
    rospy.spin()