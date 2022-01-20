#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from rospy.exceptions import ROSInterruptException
from turtlesim.msg import Pose as tPose
import numpy as np
import math
from std_srvs.srv import Empty, EmptyResponse

robot_state = {
    "idle":0,
    "run":1
}
class Node :
    def __init__(self):
        self.state = robot_state["idle"]
        self.goal_position = np.array([0,0])
        self.current_position = np.array([0,0])
        self.current_orientation = 0
        self.cmd_vel_msg = Twist()
        self.isThere_msg = Bool()
        self.isThere_msg.data = False
        self.isAtEnd = Bool()
        self.isAtEnd.data = False
        self.timer = None
    def callback_current_pose(self,msg):
        self.current_position  = np.array([msg.x,msg.y])
        self.current_orientation = msg.theta
    def callback_goal_pose(self,msg):
        self.goal_position = np.array([msg.x,msg.y])
    def callback_isAtEnd(self,msg):
        self.isAtEnd = msg
    def callback_shutDownTimer(self):
        self.timer.shutdown()
    def control(self):
        dp = self.goal_position-self.current_position
        if np.linalg.norm(dp)<1:
            v = 0
        else:
            v = 1
        e = math.atan2(dp[1],dp[0])-self.current_orientation
        K = 5
        w = K*math.atan2(math.sin(e),math.cos(e))
        return v,w
    def publish(self,event):
        distance = np.linalg.norm(self.goal_position-self.current_position)
        if self.state == robot_state["run"]:
            v,w = self.control()
            self.cmd_vel_msg.linear.x = v
            self.cmd_vel_msg.angular.z = w
            pub_cmd.publish(self.cmd_vel_msg)
            if distance<1:
                self.isThere_msg.data = True
                pub_trigger.publish(self.isThere_msg)
                if self.isAtEnd.data:
                    self.state = robot_state["idle"]
        else:
            if not self.isAtEnd.data:
                self.state = robot_state["run"]
    def service_show_state(self,request):
        print(self.state)
        response =  EmptyResponse()
        return response


if __name__=='__main__':
    rospy.init_node('via_point_follower')
    node = Node()
    srv_show_state = rospy.Service('/show_state',Empty,node.service_show_state)
    rospy.Subscriber('/turtle1/pose',tPose,node.callback_current_pose)
    rospy.Subscriber('/goal_pose',tPose,node.callback_goal_pose)
    rospy.Subscriber('/isAtEnd',Bool,node.callback_isAtEnd)
    pub_trigger = rospy.Publisher('/isThere',Bool,queue_size=10)
    pub_cmd = rospy.Publisher('turtle1/cmd_vel',Twist,queue_size=10)
    node.timer = rospy.Timer(rospy.Duration(0.1), node.publish)
    rospy.on_shutdown(node.callback_shutDownTimer)
    rospy.spin()
