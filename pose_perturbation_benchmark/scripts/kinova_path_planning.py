#!/usr/bin/env python

'''
Author(s): Yi Herng Ong
Purpose: Use MoveIt interface to plan paths for Kinova Jaco arm for executing new pose hallucination benchmark 
Date: June 2019
'''

import rospy
import moveit_commander
import sys, os
import moveit_msgs.msg
# from moveit_msgs.msg import PickupAction, PickupGoal, PlaceAction, PlaceGoal
import tf.transformations
# import std_msgs.msg
import geometry_msgs.msg
import pdb
import tf, math
from sensor_msgs.msg import JointState
import time
from std_msgs.msg import String
from std_msgs.msg import Bool
import random
import numpy as np
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import PlanningScene, PlanningSceneComponents, AllowedCollisionEntry, AllowedCollisionMatrix
from moveit_msgs.srv import GetPlanningScene, ApplyPlanningScene
from ppb_Benchmark import *
import serial
import time
from qrtest import main_f
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
			rospy.wait_for_service("/apply_planning_scene", 10.0)
			rospy.wait_for_service("/get_planning_scene", 10.0)
			self.aps = rospy.ServiceProxy('/apply_planning_scene', ApplyPlanningScene)
			self.gps = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
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
			# self.pubPlanningScene = rospy.Publisher("planning_scene" PlanningScene)
			# rospy.wait_for_service("/get_planning_scene", 10.0)
			# get_planning_scene = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
			# request = PlanningSceneComponents(components=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX)
			# response = get_planning_scene(request)


	def allow_collision(self):
		# self.pubPlanningScene = rospy.Publisher("planning_scene", PlanningScene)
		ps = PlanningScene()
		psc = PlanningSceneComponents()
		acm = AllowedCollisionMatrix()
		ace = AllowedCollisionEntry()
		is_allow = Bool()
		is_allow.data = False
		ace.enabled = [False]
		# get scene components 
		# psc.ALLOWED_COLLISION_MATRIX
		getScene = self.gps(psc) # make it empty for now
		# ps = getScene
		ps.allowed_collision_matrix.entry_names = ["cube"]
		ps.allowed_collision_matrix.entry_values = [ace]
		ps.allowed_collision_matrix.default_entry_names = ["cube"]
		ps.allowed_collision_matrix.default_entry_values = [1]
		ps.is_diff = 1
		# print "gripper name", self.gripper
		applyScene = self.aps(ps)
		print ps
		# ps.robot_state =  getScene.robot_state
		# ps.fixed_frame_transforms = getScene.fixed_frame_transforms
		# ps.allowed_collision_matrix = getScene.allowed_collision_matrix
		# ps.link			

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
		self.group.set_planning_time(10)
		self.group.go(wait=True)
		self.group.execute(self.plan, wait=True)
		self.group.stop()
		self.group.clear_pose_targets()
		rospy.sleep(2)

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

	def get_Object(self, sizes, poses, shape):
		box_pose = PoseStamped()
		box_pose.header.frame_id = self.robot.get_planning_frame()
		box_pose.pose.position.x = poses[0]
		box_pose.pose.position.y = poses[1]
		box_pose.pose.position.z = poses[2]
		box_pose.pose.orientation.w = poses[3]
		box_name = shape
		self.scene.add_box(box_name, box_pose, size=(sizes[0], sizes[1], sizes[2]))

	def get_Robot_EEposemsg(self):
		return self.group.get_current_pose()

	def get_Robot_EErpy(self):
		return self.group.get_current_rpy()

	def get_Robot_EE6Dpose(self):
		xyz = [self.get_Robot_EEposemsg().pose.position.x, self.get_Robot_EEposemsg().pose.position.y, self.get_Robot_EEposemsg().pose.position.z]
		rpy = self.get_Robot_EErpy()

		return xyz + rpy 


def get_random_translation():
	translation = ['x', 'y', 'z']
	import random
	return random.choice(translation)


def readfile(filename):
	import csv
	pose = []
	all_Poses = []
	with open(filename) as csvfile:
		csv_reader = csv.reader(csvfile, delimiter=',')
		for row in csv_reader:
			temp = row
			for each in temp:
				try:
					pose.append(float(each))
				except:
					raise ValueError
					pass
			all_Poses.append(pose)	

	return all_Poses # make sure all poses are float 

def find_pose(poses, target_pose, axis):
	for pose in poses:
		if pose[axis] == target_pose:
			return poses.index(pose)
		else:
			return None


def main():
	############################### BENCHMARK PIPELINE ######################################
	# Open reset port
	# ser = serial.Serial('/dev/ttyACM0')

	# Set initial pose and base pose
	initial_pose = [0.03, -0.54, 0.01, 90, 180, 0] # about 10 cm away from object, treat it as home pose for the benchmark
	lift_pose = [-0.24, -0.8325, 0.50, 90, 180, 0] # lifting object
	base_pose = [0.03, (-0.54-0.104), 0.01, 90, 180, 0] # make it closer to object

	# Get pose extremes and increments based on object size and type

	# Hand geometry
	hand_width = 0.175
	hand_height = 0.08
	hand_depth = 0.08
	table_to_hand_distance = 0.0254 # the height of the gripper

	# small retangular block (can apply on other small objects as well)
	obj_width = 0.04375
	obj_depth = 0.04375
	obj_height = 0.04375 + table_to_hand_distance
	trans_inc = 0.02
	orien_inc = 15

	ppB = ppBenchmark(hand_width, hand_depth, hand_height, table_to_hand_distance, obj_width, obj_depth, obj_height, trans_inc, orien_inc, initial_pose)
	ppB.get_X_limits()
	ppB.get_Y_limits()
	ppB.get_Z_limits()
	ppB.get_z()
	ppB.get_y()
	ppB.get_x()
	ppB.get_R_limits()
	ppB.get_P_limits()
	ppB.get_W_limits()
	ppB.get_r()
	ppB.get_p()
	ppB.get_w()
	ppB.sampling_limits()
	ppB.save_poses_into_csv("kg_s_rectblock")
	limits = ppB.get_actual_limits()
	ranges = ppB.get_actual_ranges()
	# Test limits and conduct binary search

	# Initialization
	Robot = robot("kinova")
	Robot.scene.remove_world_object()

	# Set planner type
	Robot.planner_type("RRT")
	Robot.get_Object([0.13125, 0.13125, 0.93125], [0.0, -0.66, (-0.05 + 0.465625 + 0.01), 1.0], "cube") # get cube
	rospy.sleep(2)

	# Move to home pose of the benchmark
	Robot.move_to_Goal(initial_pose)

	total_axis_count = 0

	while True:
		# Randomly pick an axis. Conduct binary search
		chosen_axis = random.randint(0,5)
		chosen_axis_poses  = ppB.all_limits[chosen_axis][:]
		direction = range(2)

		if chosen_axis != 2:
			chosen_dir = random.randint(0,1) # 0 - min, 1 - max
		else:
			chosen_dir = 0

		# !!!check if this direction has done!!!
		# Binary search
		if len(ranges[chosen_axis][chosen_dir]) % 2 == 1: # odd
			middle_pose = ranges[((len(ranges) - 1) / 2)]
		else: # even
			middle_pose = ranges[len(ranges) / 2]

		while_loop_check = 1

		target_pose = middle_pose[:]

		while while_loop_check:
			for pose_count, pose in enumerate(chosen_axis_poses,1):
				if pose[chosen_axis] == target_pose:
					# Check if done
					print "Pose match, looking for fail or success:", pose[-1] if pose[-1] == "s" or pose[-1] == "f" else "NOT DONE"
					if pose[-1] == "f": # fail grasp means inner pose will succeed
						try:
							target_pose = ranges[ranges.index(target_pose) - 1]
							while_loop_check = 1
							break							
						except:
							while_loop_check = 0
							done_axis_dir = 1 
							break
					elif pose[-1] == "s": # success grasp
						# check if the pose is at the last of the list
						if (ranges.index(target_pose) + 1) != len(ranges):
							target_pose = ranges[ranges.index(target_pose) + 1]
							while_loop_check = 1
							break
						else:
							while_loop_check = 0
							done_axis_dir = 1 
							break
					else:
						print "current testing limit: ", pose[chosen_axis], pose
						Robot.scene.remove_world_object()
						rospy.sleep(2)

						# Set to RRT star
						Robot.planner_type("RRT*")

						# Move to the extreme
						Robot.move_to_Goal(pose)

						# Move closer to object
						closer_pose = pose[:]
						closer_pose[1] -= 0.094
						Robot.move_to_Goal(closer_pose)

						# Grasp
						Robot.move_finger("Close")

						# Lift
						# lift_pose = pose[:]
						# lift_pose[2] = 0.3 
						Robot.move_to_Goal(lift_pose)

						# Check if grasp succeeds
						# Camera code goes in here
						if main_f() == "yes":
							# grasp suceed
							print "grasp success"
							print "find the next harder / outer pose"
							pose += ["s"]
							success_pose_index = find_pose(ppB.all_all_poses, target_pose, chosen_axis)
							ppB.all_all_poses[success_pose_index] += ["s"]
							ppB.save_poses_into_csv("kg_s_rectblock")
						else:
							# grasp failed
							print "grasp fails"
							print "find the next easier / inner pose"
							pose += ["f"]
							success_pose_index = find_pose(ppB.all_all_poses, target_pose, chosen_axis) # find pose position in the file
							ppB.all_all_poses[success_pose_index] += ["f"] # mark middle 
							ppB.save_poses_into_csv("kg_s_rectblock")

						# Open grasp
						Robot.move_finger("Open")

						# Reset object
						ser.write('r')
						while 1:
							tdata = ser.read() # Wait forever for anything
							print tdata 
							if tdata =='d':
								print 'yes'
								break 
							time.sleep(1)              # Sleep (or inWaiting() doesn't give the correct value)
							data_left = ser.inWaiting()  # Get the number of characters ready to be read
							tdata += ser.read(data_left)

						# Move back to home pose 
						Robot.get_Object([0.13125, 0.13125, 0.93125], [0.0, -0.66, (-0.05 + 0.465625 + 0.01), 1.0], "cube") # get cube for planning
						rospy.sleep(2)
						Robot.planner_type("RRT")
						Robot.move_to_Goal(initial_pose)	
						while_loop_check = 0

			if pose_count == len(chosen_axis_poses):
				while_loop_check = 0

			if done_axis_dir == 1:
				total_axis_count += 1
	################################# END ####################################
	# Compute all pose variations
	# ppB.sampling_all_Poses()

	# ppB.save_poses_into_csv("test_posefile")

		

	######################### BLOCK TEST / DEMO ##############################
	# Initialization
	Robot = robot("kinova")
	Robot.scene.remove_world_object()

	# Set planner type
	Robot.planner_type("RRT")
	
	# Read reset port

	# Robot.move_to_Goal(initial_pose)
	Robot.get_Object([0.13125, 0.13125, 0.93125], [0.0, -0.66, (-0.05 + 0.465625 + 0.01), 1.0], "cube") # get cube
	rospy.sleep(2)
	Robot.move_to_Goal(initial_pose) 
	Robot.scene.remove_world_object()
	rospy.sleep(2) # this 2 seconds is important for rrt* to work

	Robot.planner_type("RRT*")
	Robot.move_to_Goal(base_pose) # move close the object

	Robot.move_finger("Close") # close grip

	Robot.move_to_Goal(lift_pose)
	grasp_status = main_f()
	print grasp_status
	Robot.move_finger("Open") # open grip, object will drop
	
	time.sleep(3)



	ser.write('r') # reset starts

	# loop until reset is done
	while 1:
		tdata = ser.read()           # Wait forever for anything
		print tdata 
		if tdata =='d':
			print 'yes'
			break 
		time.sleep(1)              # Sleep (or inWaiting() doesn't give the correct value)
		data_left = ser.inWaiting()  # Get the number of characters ready to be read
		tdata += ser.read(data_left)	
	
	#################################### END ######################################


if __name__ == '__main__':
	main()

	# Robot = robot('kinova')
	# Robot.move_to_Goal([0.03, -0.54, 0.05, 90, 210, 0])


# pick a pose from benchmark
#
# 1. Choose a translational axis (x, y, z)
# 2. Proceed to one extreme and get result 
# 3. Determine range with binary search
# 4. Choose 
#




