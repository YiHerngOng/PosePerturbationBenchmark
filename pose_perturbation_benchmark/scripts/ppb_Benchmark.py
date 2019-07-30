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
from geometry_msgs.msg import PoseStamped

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
		self.z = [0.06]

		# rospy.init_node("expo_demo", anonymous=True)
		# self.rate = rospy.Rate(10)
		# self.marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=100)
		# self.markers_pub = rospy.Publisher("/visualization_marker_array", MarkerArray, queue_size=20)


	def samplingPoses(self, ext, inc, axis, poses):
		# min_numPose = ext[0] / inc
		if len(poses) == 0:
			poses.append(self._basePose[:])
		min_poses = deepcopy(poses)
		max_poses = deepcopy(poses)


		for pose in min_poses:
			if (axis == 3 or axis == 4) and pose[2] == self.z[0]:
				min_numPose = 1
				print ext[0]

			elif (axis == 3 or axis == 4) and pose[2] == self.z[1]:
				min_numPose = 2

			else:
				min_numPose = math.ceil(ext[0] / inc)

			for _ in range(int(min_numPose)):
				# print "min_numPose", min_numPose
				pose[axis] -= inc
				temp = pose[:]
				poses.append(temp)

		# max_numPose = ext[1] / inc
		for pose in max_poses:
			if (axis == 3 or axis == 4) and pose[2] == self.z[0]:
				max_numPose = 1

			elif (axis == 3 or axis == 4) and pose[2] == self.z[1]:
				max_numPose = 2

			else:
				max_numPose = math.ceil(ext[1] / inc)
				# if axis ==2:
					# print max_numPose

			for i in range(int(max_numPose)):
				pose[axis] += inc
				temp = pose[:]		
				poses.append(temp)

		# get z list up-down
		if axis == 2:
			minbase_z = deepcopy(self._basePose[2])
			for mdz in range(int(min_numPose)):
				minbase_z -= inc
				self.z.append(minbase_z)

			maxbase_z = deepcopy(self._basePose[2])
			for xdz in range(int(max_numPose)):
				maxbase_z += inc
				self.z.append(maxbase_z)

		return poses

	def sampling_all_Poses(self):
		self.all_Poses = []
		# self.all_Poses = self.samplingPoses((self._pExtremes[0], self._pExtremes[1]), self._pIncrement[0], 0, self.all_Poses)

		# self.all_Poses = self.samplingPoses((self._pExtremes[2],self._pExtremes[3]) , self._pIncrement[1], 1, self.all_Poses)

		self.all_Poses = self.samplingPoses((self._pExtremes[4], self._pExtremes[5]),self._pIncrement[2], 2, self.all_Poses)

		self.all_Poses = self.samplingPoses((self._oExtremes[0],self._oExtremes[1]), self._oIncrement[0], 3, self.all_Poses)

		# self.all_Poses = self.samplingPoses((self._oExtremes[2],self._oExtremes[3]), self._oIncrement[1], 4, self.all_Poses)

		# self.all_Poses = self.samplingPoses((self._oExtremes[4], self._oExtremes[5]),self._pIncrement[2], 5, self.all_Poses)
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


	
if __name__ == '__main__':
	pExt = [0.6, 0.6, 0.0, 0.4, 0.0, 0.6]
	oExt = [45, 45, 45, 45, 30, 30]
	pInc = [0.2,0.2,0.2]
	oInc = [15,15,15]

	qx = 0.73280044528
	qy = 0.0286624951588
	qz = -0.0161398685441
	qw = 0.679648051136
	angles = transformations.euler_from_quaternion([qx, qy, qz, qw])
	home_joints = [4.709939211146766, 2.8400415518300606, -2.3948526894673706e-05, 0.7500912370325066, -1.6632760154433166, 4.480076660920214, 17.446434920234484]
	# base_pose = [0.0047,-0.5159,0.072648,0.73280044528,0.0286624951588,-0.0161398685441,0.679648051136]
	base_pose = [0,-0.54,0.06,0,-0,0]

	lift_pose = [0.04250594322, -0.575295301029, 0.332918595279, 0.558853317466, 0.459234398492, 0.396013037662, 0.565650431628]
	shake_pose1 = [0.0424577485112, -0.575276284964, 0.332864738195, 0.682842077878, 0.238865152403, 0.566419484376, 0.394764617672]
	shake_pose2 = [0.0424396061673, -0.575292909309, 0.332909875925, 0.29602066861, 0.660033261061, 0.102708646288, 0.682772870014]
	ppB = ppBenchmark(base_pose, pExt, oExt, pInc, oInc)
	ppB.sampling_all_Poses()
	# for i in ppB.all_Poses:
	# 	print i
	# print ppB.z

	Robot = robot("kinova")
	# Robot.plannr_type("RRT")
	# Robot.move_to_Goal(base_pose)
	box_pose = PoseStamped()
	box_pose.header.frame_id = "kinova_move_group"
	box_pose.pose.position.x = 0.0
	box_pose.pose.position.y = -0.54
	box_pose.pose.position.z = 0.005625
	box_pose.pose.orientation.w = 1.0
	box_name = "cube"
	Robot.scene.add_box(box_name, box_pose, size=(0.13125, 0.13125, 0.13125))
	# rospy.sleep(2)
	while not rospy.is_shutdown():
		attached_objects = Robot.scene.get_attached_objects([box_name])

