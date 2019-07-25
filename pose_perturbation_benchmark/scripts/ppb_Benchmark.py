#!/usr/bin/env python

'''
Author: Yi Herng Ong
Purpose: Introduce new pose perturbation benchmark pipeline

'''
import rospy
from kinova_path_planning import *
import sys, os
import numpy as np
import pdb
from copy import deepcopy, copy
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math
from visualization_msgs.msg import Marker, MarkerArray
from tf import transformations
import serial
from object_size import *

'''
Pose perturbation benchmark:
1. Select object 
2. Select pose extremes
3. Select orientation extremes 
4. Select pose increments 
5. Select orientation increments
6. Calculate sampling poses
7. Start benchmark
8. Select from two options: start at extremes, or start from base
9. Move robot !!!
'''

# start at 2 
class ppBenchmark():
	def __init__(self, base_pose, pose_Extremes, orien_Extremes, pIncrement, oIncrement):
		self._pExtremes = pose_Extremes # [xmin, xmax, ymin, ymax, zmin, zmax] how far away between the hand and object ?
		self._oExtremes = orien_Extremes # [rmin, rmax, pmin, pmax, wmin, wmax] w -> yaw
		self._pIncrement = pIncrement # [xinc, yinc, zinc]
		self._oIncrement = oIncrement # [rinc, pinc, winc]
		self._basePose = base_pose # [x, y, z, r, p, w]
		rospy.init_node("expo_demo", anonymous=True)
		self.rate = rospy.Rate(10)
		self.marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=100)
		self.markers_pub = rospy.Publisher("/visualization_marker_array", MarkerArray, queue_size=20)


	def samplingPoses(self, ext, inc, axis, poses):
		min_numPose = ext[0] / inc
		# basepose = self._basePose[:]
		if len(poses) == 0:
			poses.append(self._basePose[:])
		min_poses = deepcopy(poses)
		max_poses = deepcopy(poses)

		for pose in min_poses:
			# if min_or_max == "min":
			for i in range(int(min_numPose)):
				pose[axis] -= inc
				temp = pose[:]
				poses.append(temp)
				# print poses
		max_numPose = ext[1] / inc
		for pose in max_poses:
			# if min_or_max == "max":
			for _ in range(int(max_numPose)):
				pose[axis] += inc
				temp = pose[:]		
				poses.append(temp)
		# print poses

		return poses

	def sampling_all_Poses(self):
		self.all_Poses = []
		self.all_Poses = self.samplingPoses((self._pExtremes[0], self._pExtremes[1]), self._pIncrement[0], 0, self.all_Poses)

		self.all_Poses = self.samplingPoses((self._pExtremes[2],self._pExtremes[3]) , self._pIncrement[1], 1, self.all_Poses)

		self.all_Poses = self.samplingPoses((self._pExtremes[4], self._pExtremes[5]),self._pIncrement[2], 2, self.all_Poses)

		self.all_Poses = self.samplingPoses((self._oExtremes[0],self._oExtremes[1]), self._oIncrement[0], 3, self.all_Poses)
		# self.all_Poses = self.samplingPoses(self._oExtremes[1], self._oIncrement[0], 3, self.all_Poses, "max")

		self.all_Poses = self.samplingPoses((self._oExtremes[2],self._oExtremes[3]), self._oIncrement[1], 4, self.all_Poses)
		# self.all_Poses = self.samplingPoses(self._oExtremes[3], self._oIncrement[1], 4, self.all_Poses, "max")

		self.all_Poses = self.samplingPoses((self._oExtremes[4], self._oExtremes[5]),self._pIncrement[2], 5, self.all_Poses)
		# self.all_Poses = self.samplingPoses(self._oExtremes[5], self._pIncrement[2], 5, self.all_Poses, "max")

	def rotation_matrix(self, r, p, w):
		r_x = np.array([[1,0,0], [0, np.cos(math.radians(p)), -np.sin(math.radians(p))], [0,np.sin(math.radians(p)), np.cos(math.radians(p))]])
		r_y = np.array([[np.cos(math.radians(r)), 0, np.sin(math.radians(r))], [0,1,0], [-np.sin(math.radians(r)), 0, np.cos(math.radians(r))]])
		r_z = np.array([[np.cos(math.radians(w)), -np.sin(math.radians(w)), 0], [np.sin(math.radians(w)), np.cos(math.radians(w)), 0], [0,0,1]])
		return np.matmul((np.matmul(r_x, r_y)), r_z)

	def pose_Plot(self):
		samples = self.all_Poses[0:10]
		x = []
		y = []
		z = []
		u = []
		v = []
		w = []
		for i in range(len(samples)):
			x.append(samples[i][0])
			y.append(samples[i][1])
			z.append(samples[i][2])
			u.append(np.matmul(self.rotation_matrix(samples[i][3], samples[i][4], samples[i][5]), np.array([1,0,0])))
			v.append(np.matmul(self.rotation_matrix(samples[i][3], samples[i][4], samples[i][5]), np.array([0,1,0])))
			w.append(np.matmul(self.rotation_matrix(samples[i][3], samples[i][4], samples[i][5]), np.array([0,0,1])))

		x = np.array(x)
		y = np.array(y)
		z = np.array(z)
		u = np.array(u)
		v = np.array(v)
		w = np.array(w)
		# print u
		# pdb.set_trace()
		fig = plt.figure()
		ax = fig.gca(projection='3d')

		ax.quiver3D(x,y,z,u[:,0],v[:,0],w[:,0],length=1, normalize=True)
		ax.quiver3D(x,y,z,u[:,1],v[:,1],w[:,1],length=1, normalize=True)
		ax.quiver3D(x,y,z,u[:,2],v[:,2],w[:,2],length=1, normalize=True)

		plt.xlim(-10,10)
		plt.ylim(-10,10)
		ax.set_zlim(-10,10)
		plt.show()



	def add_marker(self, x, y, z, r, p, w, i):
		marker = Marker()
		marker.header.frame_id = "root"
		marker.ns = "arrow"
		marker.id = i
		marker.header.stamp = rospy.Time.now()
		marker.type = Marker.ARROW
		marker.action = Marker.ADD

		# marker pose
		marker.pose.position.x = x
		marker.pose.position.y = y
		marker.pose.position.z = z
		quat = transformations.quaternion_from_euler(math.radians(r), math.radians(p), math.radians(w))
		marker.pose.orientation.x = quat[0]
		marker.pose.orientation.y = quat[1]
		marker.pose.orientation.z = quat[2]
		marker.pose.orientation.w = quat[3]

		marker.scale.x = 0.1
		marker.scale.y = 0.005
		marker.scale.z = 0.005

		marker.color.r = 0.0
		marker.color.g = 1.0
		marker.color.b = 0.0
		marker.color.a = 1.0

		return marker

	def add_marker_poses(self):
		# print len(arrow.markers)
		arrow = MarkerArray()
		arrow
		x = 0
		y = 0
		z = 0 
		poses = self.all_Poses[0:100]
		count = 0
		for i in range(len(poses)):
			# rot_mat = self.rotation_matrix(poses[i][3],poses[i][4],poses[i][5])
			for j in range(3):
				if j == 0:
					r = 90 + poses[i][4]
					p = 0 + poses[i][3]
					w = 0 + poses[i][5]
				elif j == 1:
					r = 0 + poses[i][4]
					p = -90 + poses[i][3]
					w = 0 + poses[i][5]
				elif j == 2:
					r = 0 + poses[i][4]
					p = 0 + poses[i][3]
					w = -90 + poses[i][5]
				print count
				arrow.markers.append(self.add_marker(poses[i][0],poses[i][1],poses[i][2],r,p,w,count))
				count += 1
				print poses[i][5]
		# print self.all_Poses[0:100]
		# print arrow.markers[0]
		while not rospy.is_shutdown():
			self.markers_pub.publish(arrow)


	
if __name__ == '__main__':
	pExt = [0.06, 0.06, 0.0, 0.04, 0.0, 0.04]
	oExt = [45, 45, 0, 45, 30, 30]
	pInc = [0.02,0.02,0.02]
	oInc = [15,15,15]

	qx = 0.73280044528
	qy = 0.0286624951588
	qz = -0.0161398685441
	qw = 0.679648051136
	angles = transformations.euler_from_quaternion([qx, qy, qz, qw])
	home_joints = [4.709939211146766, 2.8400415518300606, -2.3948526894673706e-05, 0.7500912370325066, -1.6632760154433166, 4.480076660920214, 17.446434920234484]
	base_pose = [0.0047,-0.5159,0.072648,0.73280044528,0.0286624951588,-0.0161398685441,0.679648051136]

	lift_pose = [0.04250594322, -0.575295301029, 0.332918595279, 0.558853317466, 0.459234398492, 0.396013037662, 0.565650431628]
	shake_pose1 = [0.0424577485112, -0.575276284964, 0.332864738195, 0.682842077878, 0.238865152403, 0.566419484376, 0.394764617672]
	shake_pose2 = [0.0424396061673, -0.575292909309, 0.332909875925, 0.29602066861, 0.660033261061, 0.102708646288, 0.682772870014]
	# ppB = ppBenchmark(base_pose, pExt, oExt, pInc, oInc)
	# ppB.sampling_all_Poses()

	# ppB.add_marker_poses()
	# define robot and move to base pose

	# print Robot.group.get_current_joint_values()
	# roll is changing y 0 15 30 45, -15, -30, -45
	# pitch - x -90, -45, -60, -120, -135
	# yaw - z 180, 165, 150, 195 210
	# base_pose = [0.0,-0.55,0.07, -90, 0, 180]
	next_pose = [0.0,-0.59,0.07, -90, 0, 180]
	lift_pose = [0.0,-0.59,0.2, -90, 0, 180]

	jpose = [0.9738466077,1.5756331639,-2.3710322135,1.0999663433,-0.4126239352,1.5473627854,-0.8734530441]
	vertical_pose = [3.1414, 3.14, 3.1414, 3.141410729179159, 3.14, 3.141534807922412, 3.14]

	Robot = robot("kinova")
	Robot.planner_type("RRT*")
	Robot.move_to_Joint(base_pose)
	# Robot.move_to_waypoint(lift_pose, "goal")
	# Robot.display_Trajectory()
	# Robot.move_to_waypoint(jpose, "pose")

	# Robot.output_trajfile(Robot.traj_pos, "trajpos5")
	# Robot.output_trajfile(Robot.traj_vel, "trajvel5")
	# Robot.output_trajfile(Robot.traj_time, "trajtime4")
	# print Robot.group.get_current_joint_values()
	# # print traj
# [-3.1417300001534465, 3.1414323853274806, 3.141459220125756, 3.1415535663807304, -3.1416293967318047, 3.141460237427801, -3.1416816724018126, 0.0, 0.0, 0.0]
 
	# filename = "waypoint.csv"
	# file = open(filename,"wb")
	# for j in traj:
	# 	joint_state = j[:]
	# 	for k in joint_state:
	# 		file.write(str(k))
	# 		file.write(',')
	# 	# file.write(str(path_count))
	# 	file.write('\n')
	# Robot.move_to_Joint([-0.5708907315,5.2319247483,2.0035223591,5.3835160097,0.6565821367,4.4928974685,-1.5458457691])
	# Robot.move_to_Joint([-1.5731004548310388, 2.8399635722906793, -3.528502071276302e-06, 0.7500470318119973, 4.619968819952383, 4.480029520452209, -1.4031059367326641])
	
	# print Robot.group.get_current_joint_values()

	####### Demo and Reset #######
	# print "Hello ! Welcome to use our automated grasp testing infrastructure."
	# ser = serial.Serial("/dev/ttyACM0")
	# reset_user = raw_input("Choose an object orientation (0 - 90)")
	# if reset_user == "0":
	# 	ser.write("d")
	# else:
	# 	ser.write(reset_user)

	# while True:
	# 	if ser.readline() == "f\n": 
	# 		break

	# user = raw_input("Choose one of the action: (L) Lift and shake (P) Pick and place (D) Drop!!!")

	# Robot = robot("kinova")
	# Robot.planner_type("RRT")
	# Robot.move_to_Goal(base_pose)
	# Robot.move_finger("Close")

	# if user == "L":
	# 	####### lift and shake #############
	# 	Robot.move_to_Goal(lift_pose)
	# 	# rospy.sleep(2)
	# 	Robot.move_to_Goal(shake_pose1)
	# 	# rospy.sleep(2)
	# 	Robot.move_to_Goal(shake_pose2)
	# 	# rospy.sleep(2)

	# 	Robot.move_to_Goal(base_pose)
	# 	# rospy.sleep(2)
	# 	Robot.move_finger("Open")

	# 	# rospy.sleep(3)
	# 	Robot.planner_type("RRT*")
	# 	Robot.move_to_Joint(home_joints)
	# 	###################################

	# if user == "P":
	# 	########### pick and place ##############
	# 	place_pose = deepcopy(base_pose)
	# 	# rospy.sleep(1)
	# 	place_pose[0] += 0.2
	# 	place_pose[1] += -0.2
	# 	Robot.move_to_Goal(lift_pose)
	# 	# rospy.sleep(1)
	# 	Robot.move_to_Goal(place_pose)
	# 	# rospy.sleep(1)
	# 	Robot.move_finger("Open")
	# 	# rospy.sleep(1)
	# 	Robot.planner_type("RRT*")
	# 	Robot.move_to_Joint(home_joints)
	# 	########################################

	# if user == "D":
	# 	########## Drop ###############
	# 	Robot.move_to_Goal(lift_pose)
	# 	Robot.move_finger("Open")
	# 	Robot.planner_type("RRT*")
	# 	Robot.move_to_Joint(home_joints)
	# 	###############################
	# rospy.sleep(3)
	# print "Sending signal to reset system ....."
	# ser.write("d")	
	# while True:
	# 	if ser.readline() == "f\n":
	# 		break