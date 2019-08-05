#!/usr/bin/env python

'''
Author: Yi Herng Ong
Purpose: Generate poses (precomputed limits & variations) for pose hallucination benchmark

'''
import rospy
# from kinova_path_planning import *
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
from geometry_msgs.msg import PoseStamped
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
	def __init__(self, hand_width, hand_depth, hand_height, table_to_hand_distance, obj_width, obj_depth, obj_height, trans_inc, orien_inc, base_pose):
		# self._pExtremes = pose_Extremes # [xmin, xmax, ymin, ymax, zmin, zmax] how far away between the hand and object ?
		# self._oExtremes = orien_Extremes # [rmin, rmax, pmin, pmax, wmin, wmax] w -> yaw
		# self._pIncrement = pIncrement # [xinc, yinc, zinc]
		# self._oIncrement = oIncrement # [rinc, pinc, winc]
		self._basePose = base_pose # [x, y, z, r, p, w]
		self.z = [base_pose[2]]
		self.y = [base_pose[1]]
		self.hand_width = hand_width
		self.hand_depth = hand_depth
		self.hand_height = hand_height
		self.table_to_hand_distance = table_to_hand_distance
		self.obj_width = obj_width
		self.obj_depth = obj_depth
		self.obj_height = obj_height
		self.trans_inc = trans_inc
		self.orien_inc = orien_inc


	def get_X_limits(self):
		x_limit = math.floor((self.hand_width - self.obj_width) / 2 * 100)
		# print x_limit
		self.x_extremes = [x_limit / 100, x_limit / 100]
		self.x_increment = self.trans_inc
		self.x_actual_limits = [self._basePose[0] - self.x_extremes[0], self._basePose[0] + self.x_extremes[1]]
		# return self.x_extremes, self.x_increment

	def get_Y_limits(self):
		self.y_increment = self.trans_inc
		y_backward_limit = math.floor(self.obj_width / 2 * 100)
		y_forward_limit = math.floor((self.hand_depth - self.obj_depth /2)*100) 
		self.y_extremes = [self.y_increment * math.floor(y_backward_limit / (self.y_increment * 100)) , self.y_increment * math.floor(y_forward_limit / (self.y_increment * 100)) ]
		# self.y_extremes[0] 
		# print self.y_increment * math.floor(y_backward_limit / (self.y_increment * 100))
		# print self.y_extremes[1]
		self.y_actual_limits = [self._basePose[1] - self.y_extremes[0], self._basePose[1] + self.y_extremes[1]]
		# print self.y_actual_limits
		# return self.y_extremes, self.y_increment

	# Note: We only look at the location of palm center
	def get_Z_limits(self):
		# palm_center_loc = ((self.hand_height / 2 ) + self.table_to_hand_distance)
		# print self.table_to_hand_distance, self.obj_height
		if self.table_to_hand_distance < self.obj_height:
			# print "here"
			z_limit = math.floor((self.obj_height - self.table_to_hand_distance)*100)
			self.z_extremes = [0.0, z_limit / 100]
			# print z_limit
		else:
			self.z_extremes = [0.0, 0.0]
		
		self.z_actual_limits = [self._basePose[2] - self.z_extremes[0], self._basePose[2] + self.z_extremes[1]]

		self.z_increment = self.trans_inc
		# return self.z_extremes, self.z_increment

	def get_R_limits(self):
		# based on reachable z 
		# length = len(self.z)
		# print self.z
		if len(self.z) > 1:
			# print "here"
			self.r_extremes = [15*(len(self.z) - 1), 15*(len(self.z) - 1)]
		else:
			self.r_extremes = [0 , 0]
		self.r_increment = self.orien_inc
		self.r_actual_limits = [self._basePose[3] - self.r_extremes[0], self._basePose[3] + self.r_extremes[1]]

		# return self.r_extremes, self.r_increment

	def get_P_limits(self):
		# based on reachable z
		# length = len(z_list)
		if len(self.z) > 1:
			self.p_extremes = [15*(len(self.z)), 15*(len(self.z))]
		else:
			self.p_extremes = [0 , 0]
		self.p_increment = self.orien_inc
		self.p_actual_limits = [self._basePose[4] - self.p_extremes[0], self._basePose[4] + self.p_extremes[1]]

		# print self.p_extremes, self.p_actual_limits
		# return self.p_extremes, self.p_increment

	def get_W_limits(self):
		# length = len(y_list)
		self.w_extremes = [30, 30]
		self.w_increment = self.orien_inc
		self.w_actual_limits = [self._basePose[5] - self.w_extremes[0], self._basePose[5] + self.w_extremes[1]]

		# return self.w_extremes, self.w_increment

	def samplingPoses(self, ext, inc, axis, poses):
		# min_numPose = ext[0] / inc
		if len(poses) == 0:
			poses.append(self._basePose[:])
		min_poses = deepcopy(poses)
		max_poses = deepcopy(poses)

		new_poses = []
		for pose in min_poses:
			if axis == 3 and pose[2] == self.z[0]:
				continue # pitch cannot have hand facing up
			elif axis == 4 and pose[2] == self.z[0]:
				# min_numPose = 1
				# print ext[0]
				continue
			elif axis == 5 and pose[1] == self.y[0]:
				min_numPose = 1
			elif axis == 5 and pose[1] == self.y[1]:
				continue
			elif (axis == 3 or axis == 4) and pose[2] == self.z[1]:
				min_numPose = 2
			else:
				min_numPose = math.ceil(ext[0] / inc)

			for i in range(int(min_numPose)):
				# print "min_numPose", min_numPose
				pose[axis] -= inc
				temp = pose[:]
				if axis == 3 or axis == 4 or axis == 5:
					new_poses.append(temp)
				else:
					poses.append(temp)


		# max_numPose = ext[1] / inc
		for pose in max_poses:
			if (axis == 3) and pose[2] == self.z[0]:
				max_numPose = 1
			elif axis == 4 and pose[2] == self.z[0]:
				continue
			elif axis == 5 and pose[1] == self.y[0]:
				max_numPose = 1
			elif axis == 5 and pose[1] == self.y[1]:
				continue
			elif (axis == 3 or axis == 4) and pose[2] == self.z[1]:
				max_numPose = 2
			else:
				max_numPose = math.ceil(ext[1] / inc)
				# if axis ==1:
					# print "here",max_numPose
			for i in range(int(max_numPose)):
				pose[axis] += inc
				temp = pose[:]		
				if axis == 3 or axis == 4 or axis == 5:
					new_poses.append(temp)
				else:
					poses.append(temp)

		if axis == 3 or axis == 4 or axis == 5:
			return new_poses
		else:
			return poses

	def get_z(self):
		min_zPose = math.ceil(self.z_extremes[0] / self.z_increment)
		max_zPose = math.ceil(self.z_extremes[1] / self.z_increment)
		minbase_z = deepcopy(self._basePose[2])
		for mdz in range(int(min_zPose)):
			minbase_z -= self.z_increment
			self.z.append(minbase_z)

		maxbase_z = deepcopy(self._basePose[2])
		for xdz in range(int(max_zPose)):
			maxbase_z += self.z_increment
			self.z.append(maxbase_z)		
		print self.z

	def get_y(self):
		min_yPose = math.ceil(self.y_extremes[0] / self.y_increment)
		max_yPose = math.ceil(self.y_extremes[1] / self.y_increment)
		minbase_y = deepcopy(self._basePose[1])
		for mdy in range(int(min_yPose)):
			minbase_y -= self.y_increment
			self.y.append(minbase_y)

		maxbase_y = deepcopy(self._basePose[1])
		for xdy in range(int(max_yPose)):
			maxbase_y += self.y_increment
			self.y.append(maxbase_y)		
		# print self.y		

	def sampling_limits(self):
		self.x_only = []
		self.y_only = []
		self.z_only = []
		self.r_only = []
		self.p_only = []
		self.w_only = []

		self.x_only = self.samplingPoses((self.x_extremes[0], self.x_extremes[1]), self.x_increment, 0, self.x_only) # left-right
		self.y_only = self.samplingPoses((self.y_extremes[0], self.y_extremes[1]), self.y_increment, 1, self.y_only) # close-far
		self.z_only = self.samplingPoses((self.z_extremes[0], self.z_extremes[1]), self.z_increment, 2, self.z_only) # up-down
		self.r_only = self.samplingPoses((self.r_extremes[0], self.r_extremes[1]), self.r_increment, 3, self.z_only) # rotate about x
		self.p_only = self.samplingPoses((self.p_extremes[0], self.p_extremes[1]), self.p_increment, 4, self.z_only) # rotate about y
		self.w_only = self.samplingPoses((self.w_extremes[0], self.w_extremes[1]), self.w_increment, 5, self.y_only) # rotate about z

		self.all_limits = [self.x_only, self.y_only, self.z_only, self.r_only, self.p_only, self.w_only]
		self.all_all_poses = self.x_only + self.y_only + self.z_only + self.r_only + self.p_only + self.w_only

		# print self.r_extremes, self.r_increment, self.z
		# print self.p_only
		# print "here",self.w_actual_limits
	# This function is to calculate combinations 
	# def sampling_all_Poses(self):
	# 	self.all_Poses = []

	# 	self.all_Poses = self.samplingPoses((self._pExtremes[0], self._pExtremes[1]), self._pIncrement[0], 0, self.all_Poses)

	# 	self.all_Poses = self.samplingPoses((self._pExtremes[2],self._pExtremes[3]) , self._pIncrement[1], 1, self.all_Poses)

	# 	self.all_Poses = self.samplingPoses((self._pExtremes[4], self._pExtremes[5]),self._pIncrement[2], 2, self.all_Poses)

	# 	self.all_Poses = self.samplingPoses((self._oExtremes[0],self._oExtremes[1]), self._oIncrement[0], 3, self.all_Poses)

	# 	self.all_Poses = self.samplingPoses((self._oExtremes[2],self._oExtremes[3]), self._oIncrement[1], 4, self.all_Poses)

	# 	self.all_Poses = self.samplingPoses((self._oExtremes[4], self._oExtremes[5]),self._pIncrement[2], 5, self.all_Poses)

		# for i in self.all_Poses:
		# 	print i
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


	# Note : markers still have problems ...
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
		poses = self.all_Poses[:]
		count = 0
		for i in range(len(poses)):
			# print len(poses)
			# rot_mat = self.rotation_matrix(poses[i][3],poses[i][4],poses[i][5])
			for j in range(3):
				# represent orientation about y
				# pointing left
				if j == 0:
					r = 0 + poses[i][3]
					p = 0 + poses[i][4]
					w = -90 + poses[i][5]
				# represent orientation about
				# pointing up 
				# elif j == 1:
				# 	r = 0 + poses[i][3]
				# 	p = -90 + poses[i][4]
				# 	w = 0 + poses[i][5]
				# # # pointing front
				# elif j == 2:
				# 	r = 0 + poses[i][3]
				# 	p = 0 + poses[i][4]
				# 	w = -90 + poses[i][5]
				# print count
				arrow.markers.append(self.add_marker(poses[i][0],poses[i][1],poses[i][2],r,p,w,count))
				count += 1
				# print poses[i][5]
		# print self.all_Poses[0:100]
		# print arrow.markers[0]
		while not rospy.is_shutdown():
			self.markers_pub.publish(arrow)

	def save_poses_into_csv(self, filename):
		f =  filename + ".csv"
		csv = open(f, "wb")
		for i in range(len(self.all_Poses)):
			temp = self.all_Poses[i]
			for j in range(len(temp)):
				csv.write(str(temp[j]))
				csv.write(",")
			csv.write("\n")
		csv.close()	



if __name__ == '__main__':
	# pExt = [0.06, 0.06, 0.0, 0.04, 0.0, 0.06]
	# oExt = [45, 45, 45, 45, 30, 30]
	# pInc = [0.02,0.02,0.02]
	# oInc = [15,15,15]

	# home_joints = [4.709939211146766, 2.8400415518300606, -2.3948526894673706e-05, 0.7500912370325066, -1.6632760154433166, 4.480076660920214, 17.446434920234484]
	# # base_pose = [0.0047,-0.5159,0.072648,0.73280044528,0.0286624951588,-0.0161398685441,0.679648051136]
	initial_pose = [0.0, -0.44, 0.01, 90, 0, 0]
	base_pose = [0.0,(-0.44-0.0144),0.01,90,0,0]

	# lift_pose = [0.04250594322, -0.575295301029, 0.332918595279, 0.558853317466, 0.459234398492, 0.396013037662, 0.565650431628]
	# shake_pose1 = [0.0424577485112, -0.575276284964, 0.332864738195, 0.682842077878, 0.238865152403, 0.566419484376, 0.394764617672]
	# shake_pose2 = [0.0424396061673, -0.575292909309, 0.332909875925, 0.29602066861, 0.660033261061, 0.102708646288, 0.682772870014]
	# ppB = ppBenchmark(initial_pose, pExt, oExt, pInc, oInc)
	# ppB.sampling_all_Poses()

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
	ppB.get_R_limits()
	ppB.get_P_limits()
	ppB.get_W_limits()
	ppB.sampling_limits()

