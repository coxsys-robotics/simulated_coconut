#!/usr/bin/python3

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Point, PoseStamped, Twist, Pose, Quaternion, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import math
import numpy as np
import tf

WHEEL_SEPARATION = 0.45
WHEEL_RADIUS = 0.08
x_odom = 0
y_odom = 0


th = 0
gain1 = 29.5
gain2 = 7.5
class odometry_broadcaster(object):
	P = np.mat(np.diag([0.01]*3))
	seq = 0
	def __init__(self):

		# self.baseFrame = rospy.get_param("~base_id", "base_footprint")
		# self.odomFrame = rospy.get_param("~odom_id", "pose_frame") 
		self.baseFrame = rospy.get_param("~base_id", "base_footprint")
		self.odomFrame = rospy.get_param("~odom_id", "odom") 
 
		self.wheelSep = float(rospy.get_param("~wheel_separation", "0.45"))
		self.wheelRad = float(rospy.get_param("~wheel_radius", "0.08"))

		self.VL = 0 
		self.VR = 0

		self.x = 0.0
		self.y = 0.0 
		self.theta = 0.0

		self.v_rx = 0.0
		self.v_ry = 0.0
		self.vth = 0.0

		self.linear_velocity_x = 0 
		self.linear_velocity_y = 0
		self.angular_velocity_z = 0

		self.odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.theta)

		self.odom_publisher = rospy.Publisher("odom", Odometry, queue_size=10)
		self.feedback_vel_publisher = rospy.Publisher("feedback_vel", Twist, queue_size=10)
		self.odom_broadcaster = tf.TransformBroadcaster()

		self.current_time = rospy.Time.now()
		self.last_time = rospy.Time.now()

	def robot_velocity_subscriber(self, v1, v2):
		self.VL = v1 
		self.VR = v2
		# self.VL = VL
		# self.VR = VR
		# VL_node = rospy.Publisher('left_vel', String, queue_size=10)
		# VR_node = rospy.Publisher('right_vel', String, queue_size=10)
		# VL_node.publish(str(VL))
		# VR_node.publish(str(VR))

	# def robot_theta_subscriber(self, theta)
		# self.theta = thetapublishpublish

	def compute_odom(self):
		self.current_time = rospy.Time.now()

		dt = (self.current_time - self.last_time).to_sec()

		robot_linear_velocity = (self.VL + self.VR)/2
		robot_angular_velocity = (self.VR - self.VL)/self.wheelSep

		self.v_rx = robot_linear_velocity
		self.vth = robot_angular_velocity

		v_wx = self.v_rx * math.cos(self.theta) - self.v_ry * math.sin(self.theta)
		v_wy = self.v_rx * math.sin(self.theta) + self.v_ry * math.cos(self.theta)

		self.x += v_wx * dt
		self.y += v_wy * dt
		
		self.theta += self.vth * dt

		self.odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.theta)
		self.odom_broadcaster.sendTransform((self.x, self.y, 0.), self.odom_quat, self.current_time, self.baseFrame, self.odomFrame)

	def publish_odom(self):
		odom = Odometry()
		odom.header.stamp = self.current_time
		odom.header.seq = self.seq
		self.seq = self.seq + 1
		odom.header.frame_id = self.odomFrame
		odom.child_frame_id = self.baseFrame

		p_cov = np.array([0.01]*36).reshape(6,6)

		# position covariance
		p_cov[0:2,0:2] = self.P[0:2,0:2]
		# orientation covariance for Yaw
		# x and Yaw
		p_cov[5,0] = p_cov[0,5] = self.P[2,0]
		# y and Yaw
		p_cov[5,1] = p_cov[1,5] = self.P[2,1]
		# Yaw and Yaw
		p_cov[5,5] = self.P[2,2]

		odom.pose.covariance = tuple(p_cov.ravel().tolist())
		odom.twist.covariance = tuple(p_cov.ravel().tolist())

		odom.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*self.odom_quat))
		odom.twist.twist = Twist(Vector3(self.v_rx, self.v_ry, 0), Vector3(0, 0, self.vth))

		twist = Twist()
		twist = Twist(Vector3(self.v_rx, self.v_ry, 0), Vector3(0, 0, self.vth))
		self.odom_publisher.publish(odom)
		self.feedback_vel_publisher.publish(twist)
		#print(odom)
		self.last_time = self.current_time

def cmd_vel_callback(data):
    # velocity->v2 = ( velocity->Vx + ((velocity->w * distanceRadius)/2.0) );
	# velocity->v1 = ( velocity->Vx - ((velocity->w * distanceRadius)/2.0) );
    
	# left_vel = data.linear.x * 1.8 
	# right_vel = data.linear.x * 1.8 
    left_wheel_vel = (data.linear.x * gain1) - (data.angular.z * gain2)
    right_wheel_vel = (data.linear.x * gain1) + (data.angular.z * gain2)
    # print(left_wheel_vel)
    left_wheel_velocity.publish(left_wheel_vel)
    right_wheel_velocity.publish(right_wheel_vel)

# def joint_callback(data):
# 	global odo
#     # ds = ( data.velocity[0] + data.velocity[1] ) / 2 
#     # dth = ( data.velocity[0] - data.velocity[1] ) / 2
#     # dx = ds * math.cos( th + (dth/2) ) 
#     # dy = ds * math.sin( th + (dth/2) ) 
#     # # x_odom = x_odom + (ds * math.cos(th + ( () )))
#     # print(dth)
# 	odo.robot_velocity_subscriber( data.velocity[0] * WHEEL_RADIUS, data.velocity[1] * WHEEL_RADIUS )
# 	v1 = data.velocity[0] * WHEEL_RADIUS
# 	v2 = data.velocity[1] * WHEEL_RADIUS
# 	print(str(v1) + "\t\t" + str(v2))
# 	odo.compute_odom()
# 	odo.publish_odom()



rospy.init_node ('diff_drive_node', anonymous = True)
rospy.Subscriber("cmd_vel", Twist, cmd_vel_callback, queue_size=1)
# rospy.Subscriber("xxx/joint_states", JointState, joint_callback, queue_size=1)
left_wheel_velocity = rospy.Publisher ("xxx/left_wheel_velocity_controller/command", Float64, queue_size=1)
right_wheel_velocity = rospy.Publisher ("xxx/right_wheel_velocity_controller/command", Float64, queue_size=1)
# odom = rospy.Publisher("odom", Odometry, queue_size=1)
# odo = odometry_broadcaster()

while not rospy.is_shutdown ():
    # marker = ellipsoid_marker()
    xx = 0
