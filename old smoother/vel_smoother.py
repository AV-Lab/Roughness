#!/usr/bin/env python
"""By Mustofa Basri"""
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import message_filters

import time
#########################
x_accel = 0.35
z_accel = 1.0
slow_const = 3 # should be slowing_const
##########################
prev_time = time.time()
last_msg = None
	
def callback(velocity, roughness):
	#print('f1')
	global roughness_value
	roughness_value = roughness.data
	global prev_time, tw, x_accel, z_accel, last_msg

	last_msg = velocity

	time_diff = min(time.time() - prev_time, 0.1)

	tw.linear.x= velocity.linear.x* roughness_value
	tw.angular.z= velocity.angular.z * roughness_value

	prev_time = time.time()
	pub.publish(tw)


print("vel_smoother node, x accel limit:", x_accel)
rospy.init_node('vel_smoother')
pub = rospy.Publisher('cmd_vel_throttled', Twist, queue_size=10)

tw = Twist()
tw.linear.y=0
tw.linear.z=0
tw.angular.x=0
tw.angular.y=0
#r = rospy.Rate(100)  # 50 hz
while not rospy.is_shutdown():
	velocity_sub = message_filters.Subscriber('joy_teleop/cmd_vel', Twist)
	roughness_sub = message_filters.Subscriber('roughness_estimate', Float64)

	# ts = message_filters.TimeSynchronizer([velocity_sub, roughness_sub], 10)
	ts = message_filters.ApproximateTimeSynchronizer([velocity_sub, roughness_sub], queue_size=10, slop=0.1, allow_headerless=True)

	ts.registerCallback(callback)

	rospy.spin()
	


# def callback_re(msg):
# 	global roughness_value
# 	roughness_value = msg
	
# def callback(msg):
# 	#print('f1')
# 	global prev_time, tw, x_accel, z_accel, last_msg

# 	last_msg = msg

# 	time_diff = min(time.time() - prev_time, 0.1)

# 	tw.linear.x= msg.linear.x* roughness_value
# 	tw.angular.z= msg.angular.z * roughness_value

# 	prev_time = time.time()
# 	pub.publish(tw)


# print("vel_smoother node, x accel limit:", x_accel)
# rospy.init_node('vel_smoother')
# pub = rospy.Publisher('cmd_vel_throttled', Twist, queue_size=10)

# tw = Twist()
# tw.linear.y=0
# tw.linear.z=0
# tw.angular.x=0
# tw.angular.y=0
# #r = rospy.Rate(100)  # 50 hz
# while not rospy.is_shutdown():
# #	rospy.Subscriber("cmd_vel", Twist, callback) 
# 	rospy.Subscriber("cmd_vel", Twist, callback)  
# 	rospy.Subscriber("roughness_estimate", Float64, callback_re)  

# 	#if last_msg is not None and time.time() - prev_time>0.02:
# 	#	pub_prev()
# 	#r.sleep()
# 	rospy.spin()
	