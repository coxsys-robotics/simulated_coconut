#!/usr/bin/env python3
import yaml
import sys,os, glob, pathlib
import numpy as np
import rospy
from sensor_msgs.msg import Imu
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
        self.data_gyro = np.zeros((num_exp,3,sample_size))
        self.data_acc = np.zeros((num_exp,3,sample_size))
        
    def load_data(self,bag_file,exp_idx,topic_name):
        bag = rosbag.Bag(bag_file)
        idx = 0
        for topic, msg, t in bag.read_messages(topics=[topic_name]):
            self.data_gyro[exp_idx][0][idx] = msg.angular_velocity.x
            self.data_gyro[exp_idx][1][idx] = msg.angular_velocity.y
            self.data_gyro[exp_idx][2][idx] = msg.angular_velocity.z
            self.data_acc[exp_idx][0][idx] = msg.linear_acceleration.x
            self.data_acc[exp_idx][1][idx] = msg.linear_acceleration.y
            self.data_acc[exp_idx][2][idx] = msg.linear_acceleration.z
            idx += 1
        bag.close()
# main
if __name__=="__main__":
    
    node = Node('imu_calibration')
    path = pathlib.Path(__file__).parent.resolve()
    path = os.path.dirname(path)  
    files = glob.glob(path+'/data/*.bag')
    num_experiment = len(files)
    num_sample_list = []
    for file in files:
        bag = rosbag.Bag(file)
        num_sample_list.append(bag.get_message_count())
    cal = Calibrator(min(num_sample_list),num_experiment)
    idx = 0
    for file in files:
        cal.load_data(file,idx,node.topic_imu)
        idx += 1
    cov_w = np.zeros((3,3))
    cov_a = np.zeros((3,3))

    for i in range(num_experiment):
         cov_w = cov_w + np.cov(cal.data_gyro[i][:][:])
         cov_a = cov_w + np.cov(cal.data_acc[i][:][:])
    cov_w = (cov_w/num_experiment)
    cov_a = (cov_a/num_experiment)

    cov_w = [float(i) for i in list(np.reshape(cov_w,(1,9))[0])]
    cov_a = [float(i) for i in list(np.reshape(cov_a,(1,9))[0])]

    dict_file = {'angular_velocity_covariance':cov_w,'linear_acceleration_covariance':cov_a}
    with open(path+'/config/'+'imu_config.yaml','w') as file:
        yaml.dump(dict_file,file)
    print('Covariance Matrices have been added to imu_config.yaml.')
    print('You can shut this node down.')

    rospy.spin()