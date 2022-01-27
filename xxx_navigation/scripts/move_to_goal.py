#!/usr/bin/env python3
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler

class Node:
    def __init__(self):
        rospy.init_node('movebase_client_py')
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.client.wait_for_server()
        self.goal = MoveBaseGoal()
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
    try:
        node = Node()
        goal_1 = (-0.442,-6.89,0)
        node.assign_goal(goal_1)
        result = node.send_and_wait()
        goal_2 = (-3.49,-7.19,0)
        node.assign_goal(goal_2)
        result = node.send_and_wait()

        if result:
            rospy.loginfo("Goal execution done!")


    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
    