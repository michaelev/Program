#!/usr/bin/env python
from duckietown_msgs.msg import Twist2DStamped, BoolStamped

import os
import rospkg
import rospy
import yaml
import time

class VehicleAvoidanceControlNode(object):

	def __init__(self):
		self.node_name = rospy.get_name()
		rospack = rospkg.RosPack()
		self.velocity = 0.34
		self.overtake = False

		self.car_cmd_pub = rospy.Publisher("~car_cmd",
				Twist2DStamped, queue_size = 1)
		self.vehicle_detected_pub = rospy.Publisher("~vehicle_detected",
				BoolStamped, queue_size = 1)

		#Publish
		self.velocity_pub = rospy.Publisher("~velocity",
				Twist2DStamped, queue_size = 1)

		#Subscribe
		self.subscriber = rospy.Subscriber("~detection",
				BoolStamped, self.callback,  queue_size = 1)
		self.sub_distance = rospy.Subscriber("~distance_robot", Twist2DStamped, self.call_dist, queue_size=1)
		self.sub_overtake_on = rospy.Subscriber("~overtake_on", BoolStamped, self.overtake_on, queue_size=1)
		self.sub_overtake_off = rospy.Subscriber("~overtake_off", BoolStamped, self.overtake_off, queue_size=1)

	def setupParam(self, param_name, default_value):
		value = rospy.get_param(param_name, default_value)
		rospy.set_param(param_name, value)
		rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
		return value

	def overtake_on(self,stats):
		if stats.data:
			self.overtake = True
			print self.overtake

	def overtake_off(self,stats):
		if stats.data:
			self.overtake = False
			print self.overtake

	def call_dist(self, dist):
		velocity = Twist2DStamped()
		Kp = 0.5
		SP = 30
		PV = dist.v
		v_g_param = self.velocity
		vo = v_g_param * 49.28302556 + 2.63080532

		if PV <= 40:
			print PV, 'cm'
			print vo, 'cm/s'
			if self.overtake:
				if PV < 30:
					rospy.set_param("lane_controller_node/d_offset", 0.28)
					velocity.v = 0.4
					self.velocity_pub.publish(velocity)
			else:
				Err = SP - PV
				P = Kp * Err
				v = vo - P
				if v > 20:
					v = 20
				if v < 8.1:
					v = 8.1
				v_s_param = (v - 2.63080532) / 49.28302556
				velocity.v = v_s_param
				self.velocity = velocity.v
				self.velocity_pub.publish(velocity)

	def callback(self, data):
		velocity = Twist2DStamped()
		if not data.data:
			print 'Vehicle Cleared'
			offset = rospy.get_param("lane_controller_node/d_offset")
			if offset == 0.28:
				time.sleep(2)
				elapsed_time = 0
				start = rospy.Time.now()
				while elapsed_time <= 2.5:
					velocity.v = 0.7
					self.velocity_pub.publish(velocity)
					elapsed_time = (rospy.Time.now() - start).to_sec()
				rospy.set_param("lane_controller_node/d_offset", -0.01)
			else:
				velocity.v = 0.34
				self.velocity = velocity.v
				self.velocity_pub.publish(velocity)
		else:
			print 'Vehicle Detected'

		#vehicle_too_close = BoolStamped()
		#vehicle_too_close.header.stamp = data.header.stamp
		#if not data.data:
		#	vehicle_too_close.data = False
		#else:
		#	vehicle_too_close.data = True
		#self.publishCmd(data.header.stamp)
		#self.vehicle_detected_pub.publish(vehicle_too_close)

	def publishCmd(self,stamp): 
		cmd_msg = Twist2DStamped()
		cmd_msg.header.stamp = stamp
		cmd_msg.v = 0
		cmd_msg.omega = 0
		self.car_cmd_pub.publish(cmd_msg)
   
if __name__ == '__main__':
	rospy.init_node('vehicle_avoidance_control_node', anonymous=False)
	controller = VehicleAvoidanceControlNode()
	rospy.spin()

