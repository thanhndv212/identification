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
from scipy import linalg, signal
import os
from os.path import dirname, join, abspath
import pandas as pd 

N1 = 32
N2 = 32
N3 = 45
N4 = -48
N5 = 45
N6 = 32

N = np.diag([N1,N2,N3,N4,N5,N6])
N[4,5] = N6


curr_data = pd.read_csv('src/thanh/curr_data.csv').to_numpy()
pos_data = pd.read_csv('src/thanh/pos_read_data.csv').to_numpy()
#Nyquist freq/0.5*sampling rate fs = 0.5 *5 kHz
b,a = signal.butter(5,50/2500, 'low')
y = np.zeros([curr_data.shape[0],curr_data.shape[1]])
t = np.linspace(0,curr_data.shape[0]-1,curr_data.shape[0])
q = np.zeros([pos_data.shape[0],pos_data.shape[1]])

plt.figure()

for i in range(curr_data.shape[1]):
	y[:,i] = signal.filtfilt(b,a,curr_data[:,i])
	q[:,i] = signal.filtfilt(b,a,pos_data[:,i])

	plt.plot(t,curr_data[:,i])
	plt.plot(t,y[:,i])

tau_T = np.dot(N,y.T)
tau = tau_T.T

#calculate vel and acc
dq = np.zeros([q.shape[0],q.shape[1]])
ddq = np.zeros([q.shape[0],q.shape[1]])


for i in range(pos_data.shape[1]):
	dq[:,i] = np.gradient(q[:,i], edge_order = 2)
	ddq[:,i] = np.gradient(dq[:,i],edge_order = 2)
	# plot(t, q[:,i])
	# plt.plot(t, dq[:,i])
	# plt.plot(t, ddq[:,i])

# plt.show()