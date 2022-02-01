#!/usr/bin/env python3
import networkx as nx
import matplotlib.pyplot as plt
import json
import os, glob, pathlib
import rospy
import math
from std_msgs.msg import Bool, UInt8

class Node :
    def __init__(self,node_name):
        self.node_name = node_name
    # initialize node
        rospy.init_node(self.node_name)
    # initialize subscribers
        #rospy.Subscriber('/topic_to_sub',sub_msg_type,self.callback_topic_to_sub)
        #self.sub_msg = sub_msg_type()
    # initialize publishers
        #self.pub_topic_to_pub = rospy.Publisher('/topic_to_pub',pub_msg_type,queue_size=50)
        #self.pub_period = 0.1
    # assign fixed time publishers 
        #self.timer = rospy.Timer(rospy.Duration(self.pub_period), self.publish)
    # assign on_shutdown behavior
        rospy.on_shutdown(self.callback_shutdown)
        
    #def callback_topic_to_sub(self,msg):
    #    self.sub_msg = msg
    #def publish(self,event):
        #topic_to_pub_msg = pub_msg_type()

        #self.pub_topic_to_pub = topic_to_pub_msg
    def callback_shutdown(self):
        print('\n\n... The node "'+self.node_name+'" has shut down...\n')

class GraphLoader:
    def __init__(self):
        pass

if __name__=="__main__":
    node = Node('graph')
    
    rospy.spin()