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


dt = 1/5000

curr_data = pd.read_csv('src/thanh/curr_data.csv').to_numpy()
pos_data = pd.read_csv('src/thanh/pos_read_data.csv').to_numpy()
#Nyquist freq/0.5*sampling rate fs = 0.5 *5 kHz
b,a = signal.butter(4,100/2500, 'low')
b1,a1 = signal.butter(5,20/2500, 'low')

y = np.zeros([curr_data.shape[0],curr_data.shape[1]])
t = np.linspace(0,curr_data.shape[0]-1,curr_data.shape[0])*dt
q = np.zeros([pos_data.shape[0],pos_data.shape[1]])

plt.figure()

for i in range(curr_data.shape[1]):
	y[:,i] = signal.filtfilt(b,a,curr_data[:,i])
	# y[:,i] = signal.medfilt(y[:,i])	
	q[:,i] = signal.filtfilt(b1,a1,pos_data[:,i])
	# q[:,i] = signal.medfilt(q[:,i])	


	# plt.plot(t,curr_data[:,i])
	# plt.plot(t,y[:,i])


N1 = 32
N2 = 32
N3 = 45
N4 = -48
N5 = 45
N6 = 32

red= np.diag([N1,N2,N3,N4,N5,N6])
q_T = np.dot(np.diag([1/N1,1/N2,1/N3,1/N4,1/N5,1/N6]),q.T)
q = q_T.T
red[4,5] = N6
# rend = np.diag([0.75,0.75,0.5,0.5,0.5,0.63])
tau_T = np.dot(red,y.T)
# tau_T = np.dot(rend,tau_T)
curr_data_T = np.dot(red,curr_data.T)
torque_data =  curr_data_T.T
tau = tau_T.T
#calculate vel and acc
dq = np.zeros([q.shape[0],q.shape[1]])
ddq = np.zeros([q.shape[0],q.shape[1]])


for i in range(pos_data.shape[1]):
	dq[:,i] = np.gradient(q[:,i], edge_order = 2)/dt
	# dq[:,i] = signal.filtfilt(b1,a1,dq[:,i])
	ddq[:,i] = np.gradient(dq[:,i],edge_order = 2)/dt
	# plt.plot(t, torque_data[:,i])
	# plt.plot(t,tau[:,i])
	# plt.plot(t, q[:,i])
	plt.plot(t, dq[:,i])
	# plt.plot(t, ddq[:,i])
plt.xlabel('time(secs)')
plt.ylabel('joint velocity(rad/s)')
plt.grid()
plt.show()