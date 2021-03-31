import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper
from pinocchio.visualize import (GepettoVisualizer, MeshcatVisualizer)
from sys import argv
# from tabulate import tabulate
import time 
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from numpy.linalg import norm, solve
from pinocchio.utils import *
from scipy import linalg
import os
from os.path import dirname, join, abspath
import pandas as pd 

isFext = False

#Load the URDF model with RobotWrapper
def loadModels(robotname, robot_urdf_file):
	"""This function create a robot model and its data by inputting a URDF file that describes the robot.
		Input: 	robotname: directory containing model of robot
			robot_urdf_file: URDF file of robot
		Output: robot: robot model and data created by Pinocchio"""
	
	pinocchio_model_dir = join(dirname(dirname(str(abspath(__file__)))),"models")
	model_path = join(pinocchio_model_dir,"others/robots")
	mesh_dir = model_path
	urdf_filename = robot_urdf_file
	urdf_dir = robotname + "/urdf"	
	urdf_model_path = join(join(model_path,urdf_dir),urdf_filename)	
	if not isFext: 
		robot = RobotWrapper.BuildFromURDF(urdf_model_path, mesh_dir)
	else: 
		robot = RobotWrapper.BuildFromURDF(urdf_model_path, mesh_dir, pin.JointModelFreeFlyer())
	return robot
def visualization(robot,q0): 
	VISUALIZER = None
	if len(argv)>1:
		opt = argv[1]
		if opt == '-g':
			VISUALIZER = GepettoVisualizer
		elif opt == '-m':
			VISUALIZER = MeshcatVisualizer
		#else:
		#    raise ValueError("Unrecognized option: " + opt)
	q = test_trajectory()
	dt = 0.02
	if VISUALIZER:
		robot.setVisualizer(VISUALIZER())
		robot.initViewer()
		robot.loadViewerModel("pinocchio")
		if q0:
			robot.display(robot.q0)
		else:
			for k in range(q.shape[0]):
				t0 = time.time()
				robot.display(q[k,:])
				t1 = time.time()
				elapsed_time = t1 - t0
				if elapsed_time < dt:
					time.sleep(dt - elapsed_time)
def test_trajectory():
	ts = 0.001
	a = 2
	b = 2
	
	N = 500
	Q = np.zeros((N,6))
	for k in range(N):
		t = k*ts
		q_i = a*t + b*t*t
		for i in range(6):
			Q[k][i] = q_i
	return Q
robot = loadModels("staubli_tx40_description", "tx40.urdf")

visualization(robot,False)
