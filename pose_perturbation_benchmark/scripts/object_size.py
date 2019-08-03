#!/usr/bin/env python

'''
Author: Yi Herng Ong
Purpose: This script takes in hand feature dimension and output 3 sizes of objects (cube, cylinder, hourglass and ellipsoid) 
'''
import numpy as np
import os, sys


'''
Input: (Hand featues/dimension) width, height, depth
Output: Dimension of each shape
units: Be consistent
'''
class Get_object_sizes(object):
	def __init__(self, robot_name, width, height, depth, table_to_hand_distance):
		self.w = width
		self.h = height
		self.d = depth
		self.table_to_hand_distance = table_to_hand_distance
		self.robot_name = robot_name

	# Note : Height of cube + 2.54 cm
	def Cube(self):
		# Max
		Xwidth = self.w * 0.75
		Xheight = Xwidth + self.table_to_hand_distance
		# Mid 
		Mwidth = self.w * 0.5
		Mheight = Mwidth + self.table_to_hand_distance
		# Min
		Swidth = self.w * 0.25
		Sheight = Swidth + self.table_to_hand_distance

		return [[Xwidth, Xheight], [MCube, Mheight], [SCube, Sheight]]

	def Cylinder(self):
		# Max
		Xradius = self.w * 0.75
		Xheight = self.h * 3
		# Mid 
		Mradius = self.w * 0.5
		Mheight = self.h * 2
		# Min
		Sradius = self.w * 0.25
		Sheight = self.h 

		return [[Xradius, Xheight], [Mradius, Mheight], [Sradius, Sheight]]

	def Hourglass(self):
		# Max 
		Xtop = self.w * 0.75
		Xhalf = self.w * 0.5
		Xheight = self.h * 3

		# Mid
		Mtop = self.w * 0.5
		Mhalf = self.w * 0.25
		Mheight = self.h * 3

		# Min
		Stop = self.w * 0.25
		Shalf = self.w * 0.1
		Sheight = self.h * 3

		return [[Xtop, Xhalf, Xheight], [Mtop, Mhalf, Mheight], [Stop, Shalf, Sheight]]

	def Ellipsoid(self):
		# Max
		Xwidth = self.w * 0.75
		Xdepth = self.d * 0.75
		Xheight = self.h * 2

		# Mid
		Mwidth = self.w * 0.5
		Mdepth = self.d * 0.5
		Mheight = self.h * 1.6

		# Min
		Swidth = self.w * 0.25
		Sdepth = self.d * 0.25
		Sheight = self.h 

		return [[Xwidth, Xdepth, Xheight], [Mwidth, Mdepth, Mheight], [Swidth, Sdepth, Sheight]]


	def csv_gen(self):
		current_directory = os.path.dirname(os.path.realpath(__file__))
		csvfile = current_directory + "/" + self.robot_name + "_objectdims" + ".csv"
		csv = open(csvfile, "wb")
		cube_dims = self.Cube()
		csv.write("cube")
		csv.write("\n")
		csv.write("side")
		csv.write("\n")
		for a in range(len(cube_dims)):
			csv.write(str(cube_dims[a]))
			csv.write("\n")

		cyl_dims = self.Cylinder()
		csv.write("cylinder")
		csv.write("\n")
		csv.write("radius")
		csv.write(",")
		csv.write("height")
		csv.write("\n")
		for b in range(len(cyl_dims)):
			each_size = cyl_dims[b]
			for c in range(len(each_size)):
				csv.write(str(each_size[c]))
				csv.write(",")
			csv.write("\n")

		hg = self.Hourglass()
		csv.write("Hourglass")
		csv.write("\n")
		csv.write("top/bottom")
		csv.write(",")
		csv.write("half")
		csv.write(",")
		csv.write("height")
		csv.write("\n")
		for d in range(len(hg)):
			each_size = hg[d]
			for e in range(len(each_size)):
				csv.write(str(each_size[e]))
				csv.write(",")			
			csv.write("\n")

		ellp = self.Ellipsoid()
		csv.write("Ellipsoid")
		csv.write("\n")
		csv.write("width")
		csv.write(",")
		csv.write("depth")
		csv.write(",")
		csv.write("height")
		csv.write("\n")
		for f in range(len(hg)):
			each_size = hg[f]
			for g in range(len(each_size)):
				csv.write(str(each_size[g]))
				csv.write(",")			
			csv.write("\n")		
		csv.close()

if __name__ == '__main__':
	# print(os.path.dirname(os.path.realpath(__file__)))
	kinova_objs = Get_object_sizes("kinova", 17.5, 8, 8)
	kinova_objs.csv_gen()












