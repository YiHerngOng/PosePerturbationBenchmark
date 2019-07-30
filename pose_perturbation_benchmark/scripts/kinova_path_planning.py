#!/usr/bin/env python
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



bp1 = [0.0, -0.44, -0.0, 90, 0, 0]
base_pose = [0.0, -0.50, 0.0, 90, 0, 0]
Robot = robot("kinova")
Robot.planner_type("RRT")
# Robot.allow_collision()

# print "current pose:", Robot.group.get_current_pose()
# print "current rpy:", Robot.group.get_current_rpy()
box_pose = PoseStamped()
box_pose.header.frame_id = Robot.robot.get_planning_frame()
box_pose.pose.position.x = 0.0
box_pose.pose.position.y = -0.54
box_pose.pose.position.z = 0.005625
box_pose.pose.orientation.w = 1.0
box_name = "cube"
Robot.scene.add_box(box_name, box_pose, size=(0.065625, 0.065625, 0.065625))

# Robot.allow_collision()

# rospy.sleep(2)
# while not rospy.is_shutdown():
# 	attached_objects = Robot.scene.get_attached_objects([box_name])

Robot.move_to_Goal(bp1)
rospy.sleep(3)
Robot.scene.remove_world_object()
rospy.sleep(2)
Robot.planner_type("RRT*")
Robot.move_to_Goal(base_pose)
