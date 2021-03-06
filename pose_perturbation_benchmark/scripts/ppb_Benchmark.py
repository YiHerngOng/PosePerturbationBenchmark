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

		self.z = []
		self.miny = []
		self.maxy = []

		self.minx = []
		self.maxx = []

		self.minr = []
		self.maxr = []

		self.minp = []
		self.maxp = []

		self.minw = []
		self.maxw = []

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
		self.x_extremes = [x_limit / 100, x_limit / 100]
		self.x_increment = self.trans_inc
		self.x_actual_limits = [self._basePose[0] - self.x_extremes[0], self._basePose[0] + self.x_extremes[1]]
		# return self.x_extremes, self.x_increment

	def get_Y_limits(self):
		self.y_increment = self.trans_inc
		y_backward_limit = math.floor(self.obj_width / 2 * 100)
		y_forward_limit = math.floor((self.hand_depth - self.obj_depth /2)*100) 
		self.y_extremes = [self.y_increment * math.floor(y_forward_limit / (self.y_increment * 100)) , self.y_increment * math.floor(y_backward_limit / (self.y_increment * 100)) ]
		self.y_actual_limits = [self._basePose[1] - self.y_extremes[0], self._basePose[1] + self.y_extremes[1]]


	# Note: We only look at the location of palm center
	def get_Z_limits(self):
		# palm_center_loc = ((self.hand_height / 2 ) + self.table_to_hand_distance)
		if self.table_to_hand_distance < self.obj_height:

			z_limit = math.floor((self.obj_height - self.table_to_hand_distance)*100)
			self.z_extremes = [0.0, z_limit / 100]

		else:
			self.z_extremes = [0.0, 0.0]
		
		self.z_actual_limits = [self._basePose[2] - self.z_extremes[0], self._basePose[2] + self.z_extremes[1]]

		self.z_increment = self.trans_inc
		# return self.z_extremes, self.z_increment

	def get_R_limits(self):
		# based on reachable z 
		# length = len(self.z)

		# if len(self.z) > 1:
		# 	# print "here"
		# 	self.r_extremes = [15*(len(self.z)), 15*(len(self.z))]
		# else:
		# 	self.r_extremes = [0 , 0]
		self.r_increment = self.orien_inc
		self.r_extremes = [45, 45]
		self.r_actual_limits = [self._basePose[3] - self.r_extremes[0], self._basePose[3] + self.r_extremes[1]]

		# return self.r_extremes, self.r_increment

	def get_P_limits(self):
		# based on reachable z
		# length = len(z_list)
		# if len(self.z) > 1:
		# 	self.p_extremes = [15*(len(self.z)), 15*(len(self.z))]
		# else:
		# 	self.p_extremes = [0 , 0]
		self.p_increment = self.orien_inc
		self.p_extremes = [45, 45]
		self.p_actual_limits = [self._basePose[4] - self.p_extremes[0], self._basePose[4] + self.p_extremes[1]]

		# return self.p_extremes, self.p_increment

	def get_W_limits(self):
		# length = len(y_list)
		self.w_extremes = [30, 30]
		self.w_increment = self.orien_inc
		self.w_actual_limits = [self._basePose[5] - self.w_extremes[0], self._basePose[5] + self.w_extremes[1]]

		# return self.w_extremes, self.w_increment

	def get_actual_limits(self):
		return [self.x_actual_limits, self.y_actual_limits, self.z_actual_limits, self.r_actual_limits, self.p_actual_limits, self.w_actual_limits]
	
	def get_actual_ranges(self):
		return [[self.minx, self.maxx], [self.miny, self.maxy], [self.z], [self.minr, self.maxr], [self.minp, self.maxp], [self.minw, self.maxw]]

	def sampling_rotation(self, ext, inc, axis, poses):
		if axis == 4: # pitch
			pose = self._basePose[:] + [0] 
			min_numPose = 3
			new_minposes = []
			for i in range(min_numPose):
				pose[axis] -= inc
				self.minp.append(pose[axis])				
				pose[2] += 0.02
				pose[1] = self._basePose[1] - 0.01 
				temp = pose[:] 
				new_minposes.append(temp)

			max_numPose = 3
			new_maxposes = []
			pose = self._basePose[:] + [0]
			for i in range(max_numPose):
				pose[axis] += inc	
				self.maxp.append(pose[axis])									
				pose[2] += 0.02
				pose[1] = self._basePose[1] - 0.01 
				temp = pose[:] 
				new_maxposes.append(temp)
			return new_minposes + new_maxposes


		if axis == 3: 
			pose = self._basePose[:] + [0]
			min_numPose = 3
			new_minposes = []
			for i in range(min_numPose):
				pose[axis] -= inc
				self.minr.append(pose[axis])			
				pose[2] += 0.02
				pose[1] = self._basePose[1] - 0.01 
				temp = pose[:]
				new_minposes.append(temp)

			max_numPose = 1
			new_maxposes = []
			pose = self._basePose[:] + [0]
			for i in range(max_numPose):
				pose[axis] += inc
				self.maxr.append(pose[axis])										
				pose[2] = 0.08
				pose[1] = self._basePose[1] - 0.01 				
				temp = pose[:] 
				new_maxposes.append(temp)
			return new_minposes + new_maxposes

		if axis == 5: 
			min_numPose = 2
			pose = self._basePose[:] + [0]
			new_minposes = []
			for j in range(min_numPose):
				pose[axis] -= inc
				self.minw.append(pose[axis])					
				# pose[1] += 0.01
				temp = pose[:] 
				new_minposes.append(temp)
			max_numPose = 2
			pose = self._basePose[:] + [0]
			new_maxposes = []				
			for j in range(max_numPose):
				pose[axis] += inc
				self.maxw.append(pose[axis])					
				# pose[1] += 0.01
				temp = pose[:] 
				new_maxposes.append(temp)			
			
			return new_minposes + new_maxposes


	def samplingPoses(self, ext, inc, axis, poses):
		# min_numPose = ext[0] / inc
		if len(poses) == 0:
			poses.append(self._basePose[:] + [0])
		min_poses = deepcopy(poses)
		max_poses = deepcopy(poses)

		new_poses = []
		for pose in min_poses:
			min_numPose = math.ceil(ext[0] / inc)
			for i in range(int(min_numPose)):
				# print "min_numPose", min_numPose
				pose[axis] -= inc
				if axis == 0:
					self.minx.append(pose[axis])
				elif axis == 1:
					self.miny.append(pose[axis])
				temp = pose[:] 
				poses.append(temp)



		# max_numPose = ext[1] / inc
		for pose in max_poses:
			max_numPose = math.ceil(ext[1] / inc)
			for i in range(int(max_numPose)):
				pose[axis] += inc
				if axis == 0:
					self.maxx.append(pose[axis])
				elif axis == 1:
					self.maxy.append(pose[axis])
				elif axis == 2:
					self.z.append(pose[axis])					
				temp = pose[:] 
				poses.append(temp)

		return poses

	def sampling_limits(self):
		self.x_only = []
		self.y_only = []
		self.z_only = []
		self.r_only = []
		self.p_only = []
		self.w_only = []

		self.x_only = self.samplingPoses((self.x_extremes[0], self.x_extremes[1]), self.x_increment, 0, self.x_only) # left-right
		# self.x_only.append(0)
		self.y_only = self.samplingPoses((self.y_extremes[0], self.y_extremes[1]), self.y_increment, 1, self.y_only) # close-far
		# self.y_only.append(0)
		self.z_only = self.samplingPoses((self.z_extremes[0], self.z_extremes[1]), self.z_increment, 2, self.z_only) # up-down
		# self.z_only.append(0)
		self.r_only = self.sampling_rotation((self.r_extremes[0], self.r_extremes[1]), self.r_increment, 3, self.r_only)  # rotate about x
		# self.r_only.append(0)
		self.p_only = self.sampling_rotation((self.p_extremes[0], self.p_extremes[1]), self.p_increment, 4, self.p_only)  # rotate about y
		# self.p_only.append(0)
		self.w_only = self.sampling_rotation((self.w_extremes[0], self.w_extremes[1]), self.w_increment, 5, self.w_only)  # rotate about z
		# self.w_only.append(0)

		self.all_limits = [self.x_only, self.y_only, self.z_only, self.r_only, self.p_only, self.w_only]
		self.all_all_poses = self.x_only + self.y_only + self.z_only + self.r_only + self.p_only + self.w_only
		self.all_rot_poses = self.r_only + self.p_only + self.w_only
		self.all_trans_poses = self.x_only + self.y_only + self.z_only

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

	def save_poses_into_csv(self, filename, poses):
		f =  filename + ".csv"
		csv = open(f, "wb")

		for i in range(len(poses)):
			temp = poses[i]
			for j in range(len(temp)):
				csv.write(str(temp[j]))
				csv.write(",")
			csv.write("\n")
		csv.close()	



if __name__ == '__main__':

	# base_pose = [0.0,(-0.44-0.0144),0.01,90,0,0]
	# initial_pose = [0.03, -0.58, 0.015, 90, 180, 0] # about 10 cm away from object, treat it as home pose for the benchmark

	initial_pose = [0.02, -0.58, 0.015, 90, 180, 0]
	hand_width = 0.175
	hand_height = 0.08
	hand_depth = 0.08
	# table_to_hand_distance = 0.0254 # the height of the gripper
	table_to_hand_distance = 0.04
	# small retangular block (can apply on other small objects as well)
	obj_width = 0.0656
	obj_depth = 0.0656
	obj_height = 0.24
	# obj_height = 0.18
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
	ppB.save_poses_into_csv("kg_m_hglass_1", ppB.all_all_poses)
	# ranges = ppB.get_actual_ranges()
	# print ranges
	# for pose in ppB.all_rot_poses:
	# 	print pose
	# print len(ppB.all_rot_poses[0])