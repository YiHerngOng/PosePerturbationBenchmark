#!/usr/bin/env python
'''
Author : Yi Herng Ong
Purpose : Import object into moveit planning scene
Reference : https://github.com/mikeferguson/moveit_python/blob/master/src/moveit_python/planning_scene_interface.py
'''
import rospy
import moveit_commander
from geometry_msgs.msg import Pose, PoseStamped, Point
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
from moveit_msgs.msg import PlanningScene, PlanningSceneComponents, ObjectColor
from moveit_msgs.srv import GetPlanningScene, ApplyPlanningScene
from shape_msgs.msg import MeshTriangle, Mesh, SolidPrimitive, Plane

class planningsceneinterface(object):
	def __init__(self, size, shape, init_from_service = True):
		self.size = size
		self.shape
		self._publish_scene = rospy.Publisher()


	def makeObject(self, shape, pose, solid):
		obj = CollisionObject()
		obj.header.stamp = rospy.Time.now()
		obj.id = shape
		obj.primitives.append(solid)
		obj.primitive_poses.append(pose)
		obj.operation = obj.ADD

		return obj

	def addsolidobject(self, shape, pose, solid, )
