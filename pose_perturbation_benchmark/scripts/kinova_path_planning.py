#!/usr/bin/env python
import rospy
import moveit_commander
import sys, os
import moveit_msgs.msg
import tf.transformations
# import std_msgs.msg
import geometry_msgs.msg
import pdb
import tf, math
from sensor_msgs.msg import JointState
import time
from std_msgs.msg import String
import random
import numpy as np
# base pose : position [0.6972, 0, 0.8] orientation [0, 90, 0]
# vary poses based on the base pose. 

class robot(object):
	def __init__(self, robot_name):
		# Initialize Moveit interface
		moveit_commander.roscpp_initialize(sys.argv)
		if robot_name == "kinova":
			rospy.init_node("kinova_move_group", anonymous=True)
			self.pub = rospy.Publisher("/if_node_finish", String, queue_size=10)
			# self.filename = rospy.get_param("~filename")
			self.robot = moveit_commander.RobotCommander()
			self.scene = moveit_commander.PlanningSceneInterface()
			self.group = moveit_commander.MoveGroupCommander("arm")
			self.group.set_planning_time(5)
			self.gripper = moveit_commander.MoveGroupCommander("gripper")
			rospy.set_param('/move_group/trajectory_execution/allowed_start_tolerance', 0.0)
			rospy.sleep(2)
			self.disp_traj = moveit_msgs.msg.DisplayTrajectory()
			self.disp_traj.trajectory_start = self.robot.get_current_state()
			self.arm_joints_sub = rospy.Subscriber("/joint_states", JointState, self.get_arm_joints)
			self.arm_joint_states = []
			self.prev_arm_js = []
			self.arm_traj = []
			# self.random_pose = [0.6972,0,0.82,0,90,0] 
			self.rate = rospy.Rate(10)	
			self.traj_pos = []
			self.traj_vel = []
			self.traj_time = []
	
	def get_arm_joints(self, msg):
		self.arm_joint_states = msg.position
		if len(self.prev_arm_js) > 0 and max(np.absolute(np.array(self.arm_joint_states) - np.array(self.prev_arm_js))) > 0.01:
			self.arm_traj.append(self.arm_joint_states)
		self.prev_arm_js = self.arm_joint_states

	def planner_type(self, planner_type):
		if planner_type == "RRT":
			self.group.set_planner_id("RRTConnectkConfigDefault")
		if planner_type == "RRT*":
			self.group.set_planner_id("RRTstarkConfigDefault")

	def move_to_Goal(self,ee_pose):
		# if ee_pose == "home":
		# 	pose_goal == ee_pose
		# else:
		pose_goal = geometry_msgs.msg.Pose()
		pose_goal.position.x = ee_pose[0]
		pose_goal.position.y = ee_pose[1]
		pose_goal.position.z = ee_pose[2]
		if len(ee_pose) == 6:
			# ee_pose = [ee_pose_xy[0], ee_pose_xy[1], 0.0868505172955, 0.990955685914, -0.130970521675, 0.00218639608306, 0.0291336691572]
			quat = tf.transformations.quaternion_from_euler(math.radians(ee_pose[3]), math.radians(ee_pose[4]), math.radians(ee_pose[5]))
			pose_goal.orientation.x = quat[0]
			pose_goal.orientation.y = quat[1]
			pose_goal.orientation.z = quat[2]
			pose_goal.orientation.w = quat[3]

		else:
			pose_goal.orientation.x = ee_pose[3]
			pose_goal.orientation.y = ee_pose[4]
			pose_goal.orientation.z = ee_pose[5]
			pose_goal.orientation.w = ee_pose[6]	

		self.group.set_pose_target(pose_goal)
		self.plan = self.group.plan()
		# self.group.set_planning_time(10)
		self.group.go(wait=True)
		self.group.execute(self.plan, wait=True)
		self.group.stop()
		self.group.clear_pose_targets()
		rospy.sleep(5)

	def move_finger(self, cmd):
		if cmd == "Close":
			self.gripper.set_named_target("Close")
			# self.gripper.go(wait=True)
		elif cmd == "Open":
			self.gripper.set_named_target("Open")
		else:
			self.gripper.set_joint_value_target(cmd)
		self.gripper.go(wait=True)
		rospy.sleep(2)

	def display_Trajectory(self):
		self.disp_traj_publisher = rospy.Publisher("/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory, queue_size=20)
		self.disp_traj.trajectory.append(self.plan)
		print self.disp_traj.trajectory
		self.disp_traj_publisher.publish(self.disp_traj)

	def move_to_Joint(self, joint_states):
		joint_goal = JointState()
		joint_goal.position = joint_states
		self.group.set_joint_value_target(joint_goal.position)
		self.plan = self.group.plan()
		# self.group.set_planning_time(5)
		self.group.go(wait=True)
		self.group.execute(self.plan, wait=True)
		self.group.stop()
		self.group.clear_pose_targets()
		rospy.sleep(2)

	def move_to_waypoint(self, p1, pose_or_goal):
		# from current move to p1
		# print "current pose", self.group.get_current_pose()
		if pose_or_goal == "goal":
			self.move_to_Goal(p1)
		else:
			self.move_to_Joint(p1)
		# print "traj points", self.plan.joint_trajectory.points
		if self.plan.joint_trajectory.points == []:
			print "No path found at this set of start and goal"
			return None
		# save traj
		else:

			# print self.plan.joint_trajectory.points
			for a in range(len(self.plan.joint_trajectory.points)):
				temp = self.plan.joint_trajectory.points[a].positions
				temp_vel = self.plan.joint_trajectory.points[a].velocities
				time_step = self.plan.joint_trajectory.points[a].time_from_start
				# print temp
				self.traj_pos.append(temp)
				self.traj_vel.append(temp_vel)
				self.traj_time.append(time_step)
			# return traj1

	def add_obj(self):
		obj_pose = geometry_msgs.msg.PoseStamped()
		obj_pose.header.frame_id = "box"
		quat_obj = tf.transformations.quaternion_from_euler(math.radians(0), math.radians(0), math.radians(0))
		obj_pose.pose.position.x = 0.5
		obj_pose.pose.position.y = 0.5
		obj_pose.pose.position.z = 0.5
		obj_pose.pose.orientation.x = quat_obj[0]
		obj_pose.pose.orientation.y = quat_obj[1]
		obj_pose.pose.orientation.z = quat_obj[2]
		obj_pose.pose.orientation.w = quat_obj[3]
		# print "obj_pose", obj_pose
		self.scene.add_box("box", obj_pose, size=(0.1,0.1,0.1))		


	def output_trajfile(self,traj, filename):
		csvfile = filename + ".csv"
		file = open(csvfile,"wb")
		for j in traj:
			try:
				joint_state = j[:]
				for k in joint_state:
					file.write(str(k))
					file.write(',')
				file.write('\n')
			except:
				joint_state = j
				file.write(str(j))
				file.write('\n')



	# def pick_and_place(self):

# if __name__ == '__main__':
	# MPNet pipeline
	# 
	# file = open("waypoints.csv", "rb")
	# lines = file.readlines()
	# waypoints = []
	# trajs_to_waypoints = []
	# Robot = robot("kinova")
	# Robot.planner_type("RRT*")
	# print Robot.group.get_current_pose()
	# sample_size = 10000
	# base_xyz = Robot.group.get_current_pose()
	# print base_xyz
	# base_rpy = Robot.group.get_current_rpy()
	# print base_rpy
	# base_pose = [base_xyz.pose.position.x, base_xyz.pose.position.y, base_xyz.pose.position.z, math.radians(0), math.radians(0), math.radians(0)]
	# Robot.move_to_Goal(base_pose)
	# path_count = 0
	# name file
	# wp_fn = str(Robot.filename) + '.csv'
	# file = open(wp_fn,"wb")
	# for i in range(sample_size):
	# 	x = random.uniform(-0.2, 0.2)
	# 	y = random.uniform(-0.3, -0.6)
	# 	z = random.uniform(0.05, 0.30)
	# 	r = random.uniform(-360, 360)
	# 	p = random.uniform(-360, 360)
	# 	w = random.uniform(-360,360)
	# 	new_pose = [x,y,z,r,p,w]
	# 	# CHECK if path feasible
	# 	traj = Robot.move_to_waypoint(new_pose)
	# 	if traj == None:
	# 		pass
	# 	else:
	# 		# write into csv
	# 		path_count += 1
	# 		for j in traj:
	# 			joint_state = j[:]
	# 			for k in joint_state:
	# 				file.write(str(k))
	# 				file.write(',')
	# 			file.write(str(path_count))
	# 			file.write('\n')


	# ee_pose = [0.0766141796649, -0.586149657477, 0.0568505172955, 0.990955685914, -0.130970521675, 0.00218639608306, 0.0291336691572]
	# ee_pose = [-(-14.2395)*0.01, -(-17.2235+60)*0.01]
	# Robot.move_to_waypoint(ee_pose)
	# ee_pose = [-(-10.2325)*0.01, -(-1.6681+60)*0.01]
	# Robot.move_to_waypoint(ee_pose)
	# ee_pose = [(-16.3469)*0.01, -(3.6912+60)*0.01]
	# ee_pose = [(0)*0.01, -(0+60)*0.01]
	
	# Robot.move_to_waypoint(ee_pose)
	# ee_pose = [(-20)*0.01, -(20+60)*0.01]
	# Robot.move_to_waypoint(ee_pose)

	# ee_pose2 = [-0.131098628652, -0.384403743486, 0.0568505172955,0.990955685914, -0.130970521675, 0.00218639608306, 0.0291336691572]
	# Robot.move_to_waypoint(ee_pose2)
	# # 
	# for i in lines:
	# 	waypoint = i.strip("\n").split(",") # formulate it such that it has xyz rpw.
	# 	traj_to_waypoint = Robot.move_to_waypoint(waypoint)
	# 	waypoints.append(waypoint)
	# 	trajs_to_waypoints.append(traj_to_waypoint)

	# if path planning done, publish a topic to if_node_finish
	# while not rospy.is_shutdown():
	# 	indicator = "NODE FINISHED"
	# 	# rospy.loginfo(indicator)
	# 	Robot.pub.publish(indicator)
	# 	Robot.rate.sleep()







	# # RRT star pipeline
	# file1 = open("start_and_goal.csv", "rb")
	# lines = file1.readlines()
	# start = lines[0].strip("\n").split(",") 
	# end = lines[-1].strip("\n").split(",")
	# Robot.planner_type("RRT*")
	# traj_to_start = Robot.move_to_waypoint(start)
	# traj_to_end = Robot.move_to_waypoint(end)

	# # RRT pipeline
	# Robot.planner_type("RRT")
	# traj_to_start = Robot.move_to_waypoint(start)
	# traj_to_end = Robot.move_to_waypoint(end)
