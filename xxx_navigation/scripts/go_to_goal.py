#!/usr/bin/env python3
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Bool, UInt8

class Node:
    def __init__(self):
        rospy.init_node('movebase_client_py')
        rospy.Subscriber('/button_go',Bool, self.callback_button_go)
        rospy.Subscriber('/goal_ID',UInt8, self.callback_goal_ID)
        self.pub_mode = rospy.Publisher('/mode',UInt8,queue_size=10)
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.client.wait_for_server()
        self.goal = MoveBaseGoal()
        self.mode = 0
        self.ID = 0
        self.result = None
    def callback_button_go(self,msg):
        if self.mode == 0:
            self.mode = 1
            mode_msg = UInt8()
            mode_msg.data = self.mode
            self.pub_mode.publish(mode_msg)
            if self.ID == 0:
                goal = (-0.442,-6.89,0)
            elif self.ID == 1:
                goal = (-3.49,-7.19,0)
            else:
                goal = (0,0,0)
            self.assign_goal(goal)
            self.result = self.send_and_wait()
            if self.result:
                rospy.loginfo("Goal execution done!")
            self.mode = 0
            mode_msg.data = self.mode
            self.pub_mode.publish(mode_msg)
            
    def callback_goal_ID(self,msg):
        self.ID = msg.data        
    def assign_goal(self,goal):
        self.goal.target_pose.header.frame_id = "map"
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.goal.target_pose.pose.position.x = goal[0]
        self.goal.target_pose.pose.position.y = goal[1]
        q = quaternion_from_euler(0,0,goal[2])

        self.goal.target_pose.pose.orientation.x = q[0]
        self.goal.target_pose.pose.orientation.y = q[1]
        self.goal.target_pose.pose.orientation.z = q[2]
        self.goal.target_pose.pose.orientation.w = q[3]
    def send_and_wait(self):
        self.client.send_goal(self.goal)
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.client.get_result()

if __name__== '__main__':
    node = Node()
    rospy.spin()
    