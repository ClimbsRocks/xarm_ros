#!/usr/bin/env python
import rospy
import actionlib
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryAction
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time

class xArmTrajAction(object):
	def __init__(self):
		self.server = actionlib.ActionServer("follow_joint_trajectory",FollowJointTrajectoryAction,self.on_goal, self.on_cancel, auto_start=False)
		self.goal_handle = None
		self.traj_pnts = None

	def start(self):
		self.server.start()

	def on_goal(self, goal_handle):
		rospy.loginfo("Received a goal!")
		self.goal_handle = goal_handle
		self.traj_pnts = self.goal_handle.get_goal().trajectory.points
		print(self.traj_pnts)
		self.goal_handle.set_accepted()
		time.sleep(1)
		self.goal_handle.set_succeeded()
		self.goal_handle = None


	def on_cancel(self, goal_handle):
		rospy.loginfo("on_cancel called!")
		self.goal_handle.set_canceled()
		self.goal_handle = None



if __name__ == '__main__':
	rospy.init_node('xarm_driver')
	server = xArmTrajAction()
	server.start()
	# if no spin(), program will not respond to "Ctrl+C"
	rospy.spin()