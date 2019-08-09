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
from qrtestRGB import main_f
import subprocess
	
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
			self.group.allow_replanning(1)
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
		if planner_type == "PRM*":
			self.group.set_planner_id("PRMstarkConfigDefault")

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
		self.group.set_planning_time(20)
		self.plan = self.group.plan()
		rospy.sleep(2)
		self.group.go(wait=True)
		# self.group.execute(self.plan, wait=True)
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
	all_Poses = []
	with open(filename) as csvfile:
		csv_reader = csv.reader(csvfile, delimiter=',')
		for row in csv_reader:
			temp = row
			pose = []
			for each in temp:
				try:
					pose.append(float(each))
				except:
					pass
			all_Poses.append(pose)	

	return all_Poses # make sure all poses are float 

def find_pose(poses, target_pose):
	print "Find target_pose index...", target_pose
	index = None
	n_poses = np.array(poses)
	tar_pose = np.array(target_pose) 
	for pose in n_poses:
		if np.max(abs(tar_pose[0:6] - pose[0:6])) < 0.00001:
			index = poses.index(list(pose))
			# print index
			return index	
		# else:
		# 	print index
	return index
			
def benchmark_feature_2():
	ser = serial.Serial('/dev/ttyACM2')

	# Set initial pose and base pose
	initial_pose = [0.02, -0.58, 0.015, 90, 180, 0] # about 10 cm away from object, treat it as home pose for the benchmark
	lift_pose = [-0.24, -0.8325, 0.30, 90, 180, 0] # lifting object
	# base_pose = [0.03, (-0.54-0.104), 0.01, 90, 180, 0] # make it closer to object

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
	ppB.get_R_limits()
	ppB.get_P_limits()
	ppB.get_W_limits()
	ppB.sampling_limits()
	# ppB.save_poses_into_csv("kg_s_cyl", ppB.all_all_poses)
	limits = ppB.get_actual_limits()
	ranges = ppB.get_actual_ranges()

	Robot = robot('kinova')
	Robot.scene.remove_world_object()
	# Set planner type
	Robot.planner_type("RRT")
	Robot.get_Object([0.13125, 0.10125, 0.93125], [0.0, -0.66, (-0.05 + 0.465625 + 0.01), 1.0], "cube") # get cube
	Robot.get_Object([0.02, 0.49, 0.50], [-0.38, -0.49, (-0.05 + 0.25 + 0.01), 1.0], "wall") # wall
	rospy.sleep(2)
	Robot.move_to_Goal(initial_pose)


	start = time.time()
	print "RUNNING BENCHMARK FEATURE 2, NOW TEST THE UNDONE POSES FROM BEGINNING..."
	# read_poses = readfile("kg_s_rectblock_2.csv") # read latest updated file
	while True:
		read_poses = readfile("kg_s_cyl_1.csv")

		for i in range(len(read_poses)):
			# if read_poses[i][-1] == 0: # undone pose
			if i == 12: # undone pose
				pose = read_poses[i][0:6]
				print "current testing pose: ", pose
				rospy.sleep(3)
				Robot.scene.remove_world_object()
				rospy.sleep(3)

				# # Set to RRT star
				Robot.planner_type("RRT*")
				rospy.sleep(3)

				# Move to the extreme
				Robot.move_to_Goal(pose)
				rospy.sleep(3)

				# Move closer to object
				closer_pose = pose[:]
				closer_pose[1] -= 0.054
				Robot.move_to_Goal(closer_pose)
				rospy.sleep(3)

				print "Start recording"
				pose_index = find_pose(read_poses, pose)
				print "Pose index: ", pose_index
				if pose_index != i:
					raise ValueError

				cmd = "rosbag record --output-name=/media/yihernong/Samsung_T5/benchmark_data/kinova_small_cylinder/kg_scyl_p{}_camera.bag /camera/color/image_raw __name:=alpha_camera &".format(pose_index)
				record = os.system(cmd)
				cmd = "rosbag record --output-name=/media/yihernong/Samsung_T5/benchmark_data/kinova_small_cylinder/kg_scyl_p{}_joints.bag /j2s7s300_driver/out/joint_state __name:=alpha_joints &".format(pose_index)
				record = os.system(cmd)


				# Grasp
				Robot.move_finger("Close")
				rospy.sleep(5)

				# Stop record
				print "Stop recording"
				# record_data.terminate()
				kill_node = os.system("rosnode kill /alpha_camera")
				kill_node = os.system("rosnode kill /alpha_joints")

				# Lift
				# lift_pose = pose[:]
				# lift_pose[2] = 0.3 
				Robot.move_to_Goal(lift_pose)
				rospy.sleep(3)

				# Check if grasp succeeds
				# Camera code goes in here
				print "Detecting grasp..."
				if main_f() == "yes":
					# grasp suceed
					print "grasp success"
					# print "find the next harder / outer pose"
					# pose += ["s"]
					success_pose_index = find_pose(read_poses, pose)
					if success_pose_index == None:
						raise ValueError 
					print "success_pose_index found: ", success_pose_index
					read_poses[success_pose_index][-1] = 1
					ppB.save_poses_into_csv("kg_s_cyl_1", read_poses)
				else:
					# grasp failed
					print "grasp fails"
					# print "find the next easier / inner pose"
					# pose += ["f"]
					fail_pose_index = find_pose(read_poses, pose) # find pose position in the file
					if fail_pose_index == None:
						raise ValueError 
					print "fail_pose_index found: ", fail_pose_index
					read_poses[fail_pose_index][-1] = -1 # mark middle 
					ppB.save_poses_into_csv("kg_s_cyl_1", read_poses)


				# Open grasp
				Robot.move_finger("Open")
				rospy.sleep(3)


				# Reset object
				print "Resetting object..."
				# ser.write('r')

				rospy.sleep(3)
				print "Move back to home pose"
				# Move back to home pose 
				Robot.get_Object([0.13125, 0.10125, 0.93125], [0.0, -0.66, (-0.05 + 0.465625 + 0.01), 1.0], "cube") # get cube
				Robot.get_Object([0.02, 0.49, 0.50], [-0.38, -0.49, (-0.05 + 0.25 + 0.01), 1.0], "wall") # wall
				rospy.sleep(3)
				Robot.planner_type("RRT")
				rospy.sleep(3)
				Robot.move_to_Goal(initial_pose)	

				# reset ping 
				# ser.write('p')
				# while 1:
				# 	tdata = ser.read() # Wait forever for anything
				# 	print tdata 
				# 	if tdata =='d':
				# 		print 'yes'
				# 		break 
				# 	time.sleep(1)              # Sleep (or inWaiting() doesn't give the correct value)
				# 	data_left = ser.inWaiting()  # Get the number of characters ready to be read
				# 	tdata += ser.read(data_left)

				break
		if i == len(read_poses) -1:
			print "Done"
			break	
	print "total time used: ", time.time() - start
	# Robot.planner_type("RRT*")
	# Robot.move_to_Goal([0.07, -0.644, 0.01, 90, 180, 0]) # move close the object

def check_pose_f_or_s(poses, target_pose):
	n_poses = np.array(poses)
	tar_pose = np.array(target_pose) 
	f_or_s = None	
	index = None
	for pose in n_poses:
		# print tar_pose, pose[0:6]
		if np.max(abs(tar_pose[0:6] - pose[0:6])) < 0.00001:
			# print pose
			f_or_s = pose[-1] 
			# print "Read from doc: ", f_or_s 
			return f_or_s
	return f_or_s

def find_range_index(ranges, pose):
	index = None
	for r in ranges:
		if abs(r - pose) < 0.00001:
			index = ranges.index(r) 
			return index

	return index

def find_target_index(read_poses, target_pose, chosen_axis):
	index = None
	for pose in read_poses:
		if abs(pose[chosen_axis] - target_pose) < 0.00001:
			index = read_poses.index(pose)
			return index
	return index


def main():
	############################### BENCHMARK PIPELINE ######################################
	# Open reset port
	ser = serial.Serial('/dev/ttyACM2')

	# Set initial pose and base pose
	initial_pose = [0.02, -0.58, 0.015, 90, 180, 0] # about 10 cm away from object, treat it as home pose for the benchmark
	lift_pose = [-0.24, -0.8325, 0.30, 90, 180, 0] # lifting object
	# base_pose = [0.03, (-0.54-0.104), 0.01, 90, 180, 0] # make it closer to object

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
	ppB.get_R_limits()
	ppB.get_P_limits()
	ppB.get_W_limits()
	ppB.sampling_limits()
	# ppB.save_poses_into_csv("kg_s_cyl", ppB.all_all_poses)
	limits = ppB.get_actual_limits()
	ranges = ppB.get_actual_ranges()

	# Initialization
	Robot = robot("kinova")
	Robot.scene.remove_world_object()

	# Set planner type
	Robot.planner_type("RRT")
	Robot.get_Object([0.13125, 0.10125, 0.93125], [0.0, -0.66, (-0.05 + 0.465625 + 0.01), 1.0], "cube") # get cube
	Robot.get_Object([0.02, 0.49, 0.50], [-0.38, -0.49, (-0.05 + 0.25 + 0.01), 1.0], "wall") # wall
	rospy.sleep(2)

	# Move to home pose of the benchmark
	print "Pose variation computed, Robot is moving to initial pose..."
	Robot.move_to_Goal(initial_pose)

	total_axis_count = 0
	record_done_axis = []

	while True:
		if total_axis_count == 11:
			print "All axis limits are computed, DONE..."
			break
		print "Total axis count: ", total_axis_count 

		# Randomly pick an axis. Conduct binary search
		chosen_axis = random.randint(0,5)
		chosen_axis_poses  = ppB.all_limits[chosen_axis][:]

		if chosen_axis != 2:
			chosen_dir = random.randint(0,1) # 0 - min, 1 - max
		else:
			chosen_dir = 0

		# !!!check if this direction has done!!!
		# Binary search
		if len(ranges[chosen_axis][chosen_dir]) % 2 == 1: # odd
			pose_arr = ranges[chosen_axis][chosen_dir]
			middle_pose = pose_arr[((len(ranges[chosen_axis][chosen_dir]) - 1) / 2)]
		else: # even
			pose_arr = ranges[chosen_axis][chosen_dir]
			middle_pose = pose_arr[len(ranges[chosen_axis][chosen_dir]) / 2]

		while_loop_check = True
		done_axis_dir = 0
		target_pose = middle_pose

		while while_loop_check:
			# check if all axis limits are computed
			read_poses = readfile("kg_s_cyl_1.csv")

			for pose_count, pose in enumerate(chosen_axis_poses,1):
				if abs(pose[chosen_axis] - target_pose) < 0.00001:
					# Check if done
					print "Target pose", target_pose, pose[chosen_axis]
					print "Current chosen limit: ", pose[chosen_axis], pose
					# print "Pose match, looking for fail or success:", pose[-1] if (pose[-1] == 1) or (pose[-1] == -1) else "NOT DONE"

					if check_pose_f_or_s(read_poses, pose) == -1: # fail grasp means inner pose will succeed
						print "Pose found was failed, looking for next inner pose"
						range_index = find_range_index(ranges[chosen_axis][chosen_dir], target_pose)
						if range_index == None:
							print "range index"
							raise ValueError

						if (range_index + 1) != len(ranges[chosen_axis][chosen_dir]):
							r_f = ranges[chosen_axis][chosen_dir]
							outer_failure_pose = r_f[range_index + 1] # set outer pose as failure
							outer_fpose_index = find_target_index(read_poses, outer_failure_pose, chosen_axis)
							if outer_fpose_index == None:
								print "outer_fpose_index"
								raise ValueError
							read_poses[outer_fpose_index][-1] = -1
							ppB.save_poses_into_csv("kg_s_cyl_1", read_poses)

						if (range_index - 1) >= 0:	
							r = ranges[chosen_axis][chosen_dir]
							target_pose = r[range_index - 1] # get inner pose
							# check if inner pose is success
							target_index = find_target_index(read_poses, target_pose, chosen_axis)
							if target_index == None:
								print "target index"
								raise ValueError
							if read_poses[target_index][-1] == 1:
								while_loop_check == False
								done_axis_dir = 1
								print "This axis and dir is done"
								break
							else:
								while_loop_check = True
								print "next pose found"
								break						
						else:
							while_loop_check = False
							done_axis_dir = 1
							print "This axis and dir is done"
							break
					elif check_pose_f_or_s(read_poses, pose) == 1: # success grasp
						print "Pose found was success, looking for next outer pose"
						# check if the pose is at the last of the list
						range_index = find_range_index(ranges[chosen_axis][chosen_dir], target_pose)
						if range_index == None:
							raise ValueError			
						if (range_index + 1) < len(ranges[chosen_axis][chosen_dir]):
							r = ranges[chosen_axis][chosen_dir]
							target_pose = r[range_index + 1] # get outer pose
							# check if outer pose is fail
							target_index = find_target_index(read_poses, target_pose, chosen_axis)
							if target_index == None:
								print "target index"
								raise ValueError
							if read_poses[target_index][-1] == -1:
								while_loop_check == False
								done_axis_dir = 1
								print "This axis and dir is done"
								break
							else:							
								while_loop_check = True
								print "next pose found"
								break
						else:
							while_loop_check = False
							done_axis_dir = 1 
							print "This axis and dir is done"
							break

					elif check_pose_f_or_s(read_poses, pose) == 0: # if not tested, NOT DONE
						print "Pose found has not been tested. Running..."
						rospy.sleep(3)
						Robot.scene.remove_world_object()
						rospy.sleep(2)

						# Set to RRT star
						Robot.planner_type("RRT*")
						rospy.sleep(3)

						# Move to the extreme
						print "Move to the chosen limit pose"
						Robot.move_to_Goal(pose[0:6])

						# Move closer to object
						print "Approaching..."
						closer_pose = pose[0:6][:]
						closer_pose[1] -= 0.054
						print "closer pose", closer_pose
						rospy.sleep(3)
						Robot.move_to_Goal(closer_pose)
						rospy.sleep(3)


						# Start recording
						print "Start recording..."
						# get pose index 
						pose_index = find_pose(read_poses, pose)
						print "Pose index: ", pose_index
						# record_data = subprocess.Popen(["rosbag", "record", "--output-name=/media/yihernong/Samsung_T5/benchmark_data/kinova_sblock_a{}_d{}.bag".format(chosen_axis,chosen_dir), "/camera/color/image_raw", "/j2s7s300_driver/out/joint_state"])
						cmd = "rosbag record --output-name=/media/yihernong/Samsung_T5/benchmark_data/kinova_small_cylinder/kg_scyl_p{}_camera.bag /camera/color/image_raw __name:=alpha_camera &".format(pose_index)
						record = os.system(cmd)
						cmd = "rosbag record --output-name=/media/yihernong/Samsung_T5/benchmark_data/kinova_small_cylinder/kg_scyl_p{}_joints.bag /j2s7s300_driver/out/joint_state __name:=alpha_joints &".format(pose_index)
						record = os.system(cmd)

						rospy.sleep(5)
						# Grasp
						print "Grasping..."
						Robot.move_finger("Close")

						rospy.sleep(5)
						# Stop record
						print "Stop recording"
						# record_data.terminate()
						kill_node = os.system("rosnode kill /alpha_camera")
						kill_node = os.system("rosnode kill /alpha_joints")

						# Lift
						print "Lifting..."
						rospy.sleep(3)
						Robot.move_to_Goal(lift_pose)

						# Check if grasp succeeds
						# Camera code goes in here
						print "Detecting grasp..."
						if main_f() == "yes":
							# grasp suceed
							print "grasp success"
							# print "find the next harder / outer pose"
							# pose += ["s"]
							success_pose_index = find_pose(read_poses, pose)
							if success_pose_index == None:
								print "Success_pose_index not found"
								raise ValueError 
							print "success_pose_index found: ", success_pose_index
							read_poses[success_pose_index][-1] = 1
							ppB.save_poses_into_csv("kg_s_cyl_1", read_poses)
						else:
							# grasp failed
							print "grasp fails"
							# print "find the next easier / inner pose"
							# pose += ["f"]
							fail_pose_index = find_pose(read_poses, pose) # find pose position in the file
							if fail_pose_index == None:
								print "Fail_pose_index not found"
								raise ValueError 
							print "fail_pose_index found: ", fail_pose_index
							read_poses[fail_pose_index][-1] = -1
							ppB.save_poses_into_csv("kg_s_cyl_1", read_poses)

						# Open grasp
						print "Open grasp"
						Robot.move_finger("Open")
						rospy.sleep(3)


						# Reset object
						print "Resetting object..."
						ser.write('r')
						# while 1:
						# 	tdata = ser.read() # Wait forever for anything
						# 	print tdata 
						# 	if tdata =='d':
						# 		print 'yes'
						# 		break 
						# 	time.sleep(1)              # Sleep (or inWaiting() doesn't give the correct value)
						# 	data_left = ser.inWaiting()  # Get the number of characters ready to be read
						# 	tdata += ser.read(data_left)

						# Move back to home pose 
						print "Done resetting, move to home pose..."
						Robot.get_Object([0.13125, 0.10125, 0.93125], [0.0, -0.66, (-0.05 + 0.465625 + 0.01), 1.0], "cube") # get cube for planning
						Robot.get_Object([0.02, 0.49, 0.60], [-0.38, -0.49, (-0.05 + 0.30 + 0.01), 1.0], "wall") # wall
						rospy.sleep(3)
						Robot.planner_type("RRT")
						rospy.sleep(3)
						Robot.move_to_Goal(initial_pose)	

						# reset ping 
						ser.write('p')
						while 1:
							tdata = ser.read() # Wait forever for anything
							print tdata 
							if tdata =='d':
								print 'yes'
								break 
							time.sleep(1)              # Sleep (or inWaiting() doesn't give the correct value)
							data_left = ser.inWaiting()  # Get the number of characters ready to be read
							tdata += ser.read(data_left)
						
						while_loop_check = False
						break

					else:
						print "Not recorded whether the pose is done"
						raise ValueError
			if done_axis_dir == 1:
				# print "One axis and direction done: {} axis in {} direction".format(chosen_axis, "min" if chosen_dir == 0 else "max")
				code = str(chosen_axis) + str(chosen_dir)
				# total_axis_count += 1
				if code not in record_done_axis:
					print "One axis and direction done: {} axis in {} direction".format(chosen_axis, "min" if chosen_dir == 0 else "max")
					record_done_axis.append(code)
					total_axis_count += 1
				else:
					print "{} axis in {} direction has done before, find next...".format(chosen_axis, "min" if chosen_dir == 0 else "max")

				done_axis_dir = 0
				while_loop_check =False

			if pose_count == len(chosen_axis_poses):
				print "Pose count last"
				while_loop_check = False
				break


	print "ALL LIMITS ARE COMPUTED, NOW TEST THE UNDONE POSES..."
	# read_poses = readfile("kg_s_cyl.csv") # read latest updated file

	# benchmark_feature_2()

	# for i in range(len(read_poses)):
	# 	if read_poses[i][-1] == 0: # undone pose
	# 		pose = read_poses[i][:]
	# 		print "current testing pose: ", pose
	# 		rospy.sleep(3)
	# 		Robot.scene.remove_world_object()
	# 		rospy.sleep(3)

	# 		# # Set to RRT star
	# 		Robot.planner_type("RRT*")
	# 		rospy.sleep(3)

	# 		# Move to the extreme
	# 		Robot.move_to_Goal(pose)
	# 		rospy.sleep(3)

	# 		# Move closer to object
	# 		closer_pose = pose[:]
	# 		closer_pose[1] -= 0.054
	# 		Robot.move_to_Goal(closer_pose)
	# 		rospy.sleep(3)

	# 		print "Start recording"
	# 		pose_index = find_pose(read_poses, pose)
	# 		print "Pose index: ", pose_index
	# 		if pose_index != i:
	# 			print "Not match!!! ", pose_index, i
	# 			raise ValueError

	# 		cmd = "rosbag record --output-name=/media/yihernong/Samsung_T5/benchmark_data/kinova_small_cylinder/kg_scyl_p{}_camera.bag /camera/color/image_raw __name:=alpha_camera &".format(pose_index)
	# 		record = os.system(cmd)
	# 		cmd = "rosbag record --output-name=/media/yihernong/Samsung_T5/benchmark_data/kinova_small_cylinder/kg_scyl_p{}_joints.bag /j2s7s300_driver/out/joint_state __name:=alpha_joints &".format(pose_index)
	# 		record = os.system(cmd)


	# 		# Grasp
	# 		Robot.move_finger("Close")
	# 		rospy.sleep(5)

	# 		# Stop record
	# 		print "Stop recording"
	# 		# record_data.terminate()
	# 		kill_node = os.system("rosnode kill /alpha_camera")
	# 		kill_node = os.system("rosnode kill /alpha_joints")

	# 		# Lift
	# 		# lift_pose = pose[:]
	# 		# lift_pose[2] = 0.3 
	# 		Robot.move_to_Goal(lift_pose)
	# 		rospy.sleep(3)

	# 		# Check if grasp succeeds
	# 		# Camera code goes in here
	# 		print "Detecting grasp..."
	# 		if main_f() == "yes":
	# 			# grasp suceed
	# 			print "grasp success"
	# 			# print "find the next harder / outer pose"
	# 			# pose += ["s"]
	# 			success_pose_index = find_pose(read_poses, pose)
	# 			if success_pose_index == None:
	# 				print "Success_pose_index not found"
	# 				raise ValueError 
	# 			print "success_pose_index found: ", success_pose_index
	# 			read_poses[success_pose_index][-1] = 1
	# 			ppB.save_poses_into_csv("kg_s_cyl", read_poses)
	# 		else:
	# 			# grasp failed
	# 			print "grasp fails"
	# 			# print "find the next easier / inner pose"
	# 			# pose += ["f"]
	# 			fail_pose_index = find_pose(read_poses, pose) # find pose position in the file
	# 			if fail_pose_index == None:
	# 				print "Fail_pose_index not found"
	# 				raise ValueError 
	# 			print "fail_pose_index found: ", fail_pose_index
	# 			read_poses[fail_pose_index][-1] = -1 # mark middle 
	# 			ppB.save_poses_into_csv("kg_s_cyl", read_poses)


	# 		# Open grasp
	# 		Robot.move_finger("Open")
	# 		rospy.sleep(3)



	# 		# Reset object
	# 		ser.write('r')

	# 		# Move back to home pose 
	# 		Robot.get_Object([0.13125, 0.13125, 0.93125], [0.0, -0.66, (-0.05 + 0.465625 + 0.01), 1.0], "cube") # get cube for planning
	# 		Robot.get_Object([0.02, 0.49, 0.50], [-0.38, -0.49, (-0.05 + 0.25 + 0.01), 1.0], "wall") # wall
	# 		rospy.sleep(3)
	# 		Robot.planner_type("RRT")
	# 		rospy.sleep(3)
	# 		Robot.move_to_Goal(initial_pose)

	# 		# reset ping 
	# 		ser.write('p')
	# 		while 1:
	# 			tdata = ser.read() # Wait forever for anything
	# 			print tdata 
	# 			if tdata =='d':
	# 				print 'yes'
	# 				break 
	# 			time.sleep(1)              # Sleep (or inWaiting() doesn't give the correct value)
	# 			data_left = ser.inWaiting()  # Get the number of characters ready to be read
	# 			tdata += ser.read(data_left)	
	################################# BENCHMARK END ####################################
	# Compute all pose variations
	# ppB.sampling_all_Poses()

	# ppB.save_poses_into_csv("test_posefile")

		

	######################### BLOCK TEST / DEMO ##############################
	# Initialization
	# Robot = robot("kinova")
	# Robot.scene.remove_world_object()

	# # Set planner type
	# Robot.planner_type("RRT")
	
	# # Read reset port

	# # Robot.move_to_Goal(initial_pose)
	# Robot.get_Object([0.13125, 0.13125, 0.93125], [0.0, -0.66, (-0.05 + 0.465625 + 0.01), 1.0], "cube") # get cube
	# rospy.sleep(2)
	# Robot.move_to_Goal(initial_pose) 
	# Robot.scene.remove_world_object()
	# rospy.sleep(2) # this 2 seconds is important for rrt* to work

	# Robot.planner_type("RRT*")
	# Robot.move_to_Goal(base_pose) # move close the object

	# Robot.move_finger("Close") # close grip

	# Robot.move_to_Goal(lift_pose)
	# grasp_status = main_f()
	# print grasp_status
	# Robot.move_finger("Open") # open grip, object will drop
	
	# time.sleep(3)



	# ser.write('r') # reset starts

	# # loop until reset is done
	# while 1:
	# 	tdata = ser.read()           # Wait forever for anything
	# 	print tdata 
	# 	if tdata =='d':
	# 		print 'yes'
	# 		break 
	# 	time.sleep(1)              # Sleep (or inWaiting() doesn't give the correct value)
	# 	data_left = ser.inWaiting()  # Get the number of characters ready to be read
	# 	tdata += ser.read(data_left)	
	
	#################################### END ######################################


if __name__ == '__main__':
	# main()

	benchmark_feature_2()
	# read_poses = readfile("kg_s_rectblock_2.csv")
	# print np.array(read_poses[1])
	# find_pose(read_poses, [0.03, -0.58, 0.015, 90, 180, 0])
	# print check_pose_f_or_s(read_poses, read_poses[2])
	
	# Robot = robot('kinova')
	# Robot.planner_type("RRT")
	# Robot.get_Object([0.13125, 0.10125, 0.93125], [0.0, -0.66, (-0.05 + 0.465625 + 0.01), 1.0], "cube") # get cube
	# Robot.get_Object([0.02, 0.49, 0.50], [-0.38, -0.49, (-0.05 + 0.25 + 0.01), 1.0], "wall") # wall
	# rospy.sleep(2)
	# lift_pose = [-0.24, -0.8325, 0.30, 90, 180, 0] # lifting object
	# Robot.move_to_Goal(lift_pose)
	# 
	# rospy.sleep(2)
	# Robot.move_to_Goal([-0.24, -0.8325, 0.30, 90, 180, 0])
	# rospy.sleep(2)
	# Robot.move_to_Goal([0.03, -0.59, 0.015, 90, 180, 0])	
	# Robot.scene.remove_world_object()
	# rospy.sleep(2) # this 2 seconds is important for rrt* to work

	# # Robot.planner_type("RRT*")
	# Robot.move_to_Goal([0.07, -0.644, 0.01, 90, 180, 0]) # move close the object





