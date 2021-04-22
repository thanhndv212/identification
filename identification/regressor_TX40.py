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
from scipy import linalg, signal
import os
from os.path import dirname, join, abspath
import pandas as pd 
import json
import csv
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
	
#inertial parameters of link2 from urdf model
def standardParameters(njoints,fv,fs,Ia,off,Iam6,fvm6,fsm6):
	"""This function prints out the standard inertial parameters obtained from 3D design.
		Note: a flag IsFrictioncld to include in standard parameters
		Input: 	njoints: number of joints
		Output: params_std: a dictionary of parameter names and their values"""
	params_name = ['m', 'mx','my','mz','Ixx','Ixy','Iyy','Ixz', 'Iyz','Izz']
	phi = []
	params = []
	for i in range(1,njoints):
		P = model.inertias[i].toDynamicParameters()
		for k in P: 
			phi.append(k)
		for j in params_name: 
			if not isFext:
				params.append(j + str(i))
			else:
				params.append(j + str(i-1))
	if isFrictionincld:
		for k in range(1, njoints):
			phi.extend([fv[k-1],fs[k-1]])
			params.extend(['fv' + str(k),'fs' + str(k)])
	if isActuator_int:
		for k in range(1,njoints):
			phi.extend([Ia[k-1]])
			params.extend(['Ia' + str(k)])
	if isOffset:
		for k in range(1,njoints):
			phi.extend([off[k-1]])
			params.extend(['off' + str(k)])
	if isCoupling:
		phi.extend([Iam6,fvm6,fsm6])
		params.extend(['Iam6','fvm6','fsm6'])
	params_std = dict(zip(params, phi)) 
	return params_std

#generate waypoints 
def generateWaypoints(N,nq,nv,mlow,mhigh): 
	"""This function generates N random values for joints' position,velocity, acceleration.
		Input: 	N: number of samples
				nq: length of q, nv : length of v
				mlow and mhigh: the bound for random function
		Output: q, v, a: joint's position, velocity, acceleration"""
	q = np.empty((1,nq))
	v = np.empty((1,nv))
	a = np.empty((1,nv))
	for i in range(N):
		q = np.vstack((q,np.random.uniform(low = mlow, high = mhigh, size=(nq,))))		
		v = np.vstack((v,np.random.uniform(low = mlow, high = mhigh, size=(nv,))))
		a = np.vstack((a,np.random.uniform(low = mlow, high = mhigh, size=(nv,))))
	return q, v, a
#generate waypoints with base link
def generateWaypoints_fext(N, robot,mlow,mhigh): 
	"""This function generates N random values for joints' position,velocity, acceleration.
		Input: 	N: number of samples
				nq: length of q, nv : length of v
				mlow and mhigh: the bound for random function
		Output: q, v, a: joint's position, velocity, acceleration"""
	nq = robot.model.nq
	nv = robot.model.nv
	q0 = robot.q0[:7]
	v0 = np.zeros(6)
	a0 = np.zeros(6)
	q = np.empty((1,nq))
	v = np.empty((1,nv))
	a = np.empty((1,nv))
	for i in range(N):
		
		q_ = np.append(q0, np.random.uniform(low = mlow, high = mhigh, size=(nq-7,)))
		q = np.vstack((q,q_))
		
		v_ = np.append(v0,np.random.uniform(low = mlow, high = mhigh, size=(nv-6,)))
		v = np.vstack((v,v_))
		
		a_ = np.append(a0,np.random.uniform(low = mlow, high = mhigh, size=(nv-6,)))
		a = np.vstack((a,a_))
	return q, v, a
#Building regressor
def build_regressor_basic(model, data, N, nq, nv, njoints, q, v, a):
	W = np.zeros([N*nv, 10*(njoints-1)]) 
	for i in range(N):
		W_temp = pin.computeJointTorqueRegressor(model, data, q[i,:], v[i,:], a[i,:])
		for j in range(W_temp.shape[0]):
			W[j*N + i, :] = W_temp[j,:]
	return W
def build_regressor_w_friction(model, data, N, nq, nv, njoints, q, v, a):
	W = np.zeros([N*nv, 10*(njoints-1) + 2*nv]) 
	for i in range(N):
		W_temp = pin.computeJointTorqueRegressor(model, data, q[i,:], v[i,:], a[i,:])
		for j in range(W_temp.shape[0]):
			W[j*N + i, 0:10*(njoints-1)] = W_temp[j,:]
			W[j*N + i, 10*(njoints-1)+2*j] = v[i,j]
			W[j*N + i, 10*(njoints-1)+2*j + 1] = np.sign(v[i,j])
	return W
def build_regressor_full(model, data, N, nq, nv, njoints, q, v, a):
	W = np.zeros([N*nv, 10*(njoints-1) + 2*nv+nv+nv]) 
	for i in range(N):
		W_temp = pin.computeJointTorqueRegressor(model, data, q[i,:], v[i,:], a[i,:])
		for j in range(W_temp.shape[0]):
			W[j*N + i, 0:10*(njoints-1)] = W_temp[j,:]
			W[j*N + i, 10*(njoints-1)+2*j] = v[i,j]
			W[j*N + i, 10*(njoints-1)+2*j + 1] = np.sign(v[i,j])
			W[j*N + i, 10*(njoints-1)+2*nv + j] = a[i,j]
			W[j*N + i, 10*(njoints-1)+2*nv + nv + j] = 1
	return W
def add_coupling(W,model, data, N, nq, nv, njoints, q, v, a):
	W = np.c_[W, np.zeros([W.shape[0],3])]
	for i in range(N):
		for j in range(nv):
			if j == (nv-2):#joint 5
				W[j*N + i, -3] = a[i,nv-1]
				W[j*N + i, -2] = v[i,nv-1]
				W[j*N + i, -1] = np.sign(v[i,nv-2] + v[i,nv-1])	
				# W[j*N + i, -3] = -1
				# W[j*N + i, -2] = -1
				# W[j*N + i, -1] = -3
						
			if j == (nv-1):#joint 6
				W[j*N + i, -3] = a[i,nv-2]
				W[j*N + i, -2] = v[i,nv-2]
				W[j*N + i, -1] = np.sign(v[i,nv-2] + v[i,nv-1])			
				# W[j*N + i, -3] = -2
				# W[j*N + i, -2] = -2
				# W[j*N + i, -1] = -5
	return W
def get_torque_rand(model, data, N,  nq, nv,njoints, q, v, a, Ia, off, Iam6, fvm6, fsm6):
	tau = np.zeros(nv*N)
	for i in range(N):
		for j in range(nv):
			tau[j*N + i] = pin.rnea(model, data, q[i,:], v[i,:], a[i,:])[j] + v[i,j]*fv[j] + np.sign(v[i,j])*fs[j] + Ia[j]*a[i,j] + off[j]
			if j == nv-2:
				tau[j*N + i] = tau[j*N + i] +  Iam6*v[i,nv-1] + fvm6*v[i,nv-1] + fsm6*np.sign(v[i,nv-2] + v[i,nv-1])
			if j == nv -1:
				tau[j*N + i] = tau[j*N + i] +  Iam6*v[i,nv-2] + fvm6*v[i,nv-2] + fsm6*np.sign(v[i,nv-2] + v[i,nv-1])
	return tau
def  iden_model(model, data, N, nq, nv, njoints, q, v, a): 
	"""This function calculates joint torques and generates the joint torque regressor.
		Note: a flag IsFrictionincld to include in dynamic model
		Input: 	model, data: model and data structure of robot from Pinocchio
				q, v, a: joint's position, velocity, acceleration
				N : number of samples
				nq: length of q
		Output: tau: vector of joint torque
				W : joint torque regressor"""
	tau = np.empty(nv*N)
	W = np.empty([N*nv, 10*(njoints-1)]) 
	for i in range(N):
		tau_temp = pin.rnea(model, data, q[i,:], v[i,:], a[i,:])
		W_temp = pin.computeJointTorqueRegressor(model, data, q[i,:], v[i,:], a[i,:])
		for j in range(W_temp.shape[0]):
			tau[j*N + i] = tau_temp[j]
			W[j*N + i, :] = W_temp[j,:]
	if isFrictionincld:
		W = np.c_[W,np.zeros([N*nv,2*nv])]
		for i in range(N):
			for j in range(nv):
				tau[j*N + i] = tau[j*N + i] + v[i,j]*fv[j] + np.sign(v[i,j])*fs[j]
				W[j*N + i, 10*(njoints-1)+2*j] = v[i,j]
				W[j*N + i, 10*(njoints-1)+2*j + 1] = np.sign(v[i,j])
	return tau, W
def  iden_model_fext(model, data, N, nq, nv, njoints, q, v, a): 
	"""This function calculates joint torques and generates the joint torque regressor.
		Note: a flag IsFrictioncld to include in dynamic model
		Input: 	model, data: model and data structure of robot from Pinocchio
				q, v, a: joint's position, velocity, acceleration
				N : number of samples
				nq: length of q
		Output: tau: vector of joint torque
				W : joint torque regressor"""
	tau = np.empty(6*N)
	W = np.empty([N*6, 10*(njoints-1)]) 
	for i in range(N):
		tau_temp = pin.rnea(model, data, q[i,:], v[i,:], a[i,:])
		W_temp = pin.computeJointTorqueRegressor(model, data, q[i,:], v[i,:], a[i,:])
		for j in range(6):
			tau[j*N + i] = tau_temp[j]
			W[j*N + i, :] = W_temp[j,:]
	return tau, W

#Eliminate columns crspnd. non-contributing parameters
def eliminateNonAffecting(W, tol_e):
	"""This function eliminates columns which has L2 norm smaller than tolerance.
		Input: 	W: joint torque regressor
				tol_e: tolerance 
		Output: W_e: reduced regressor
				params_r: corresponding parameters to columns of reduced regressor"""
	col_norm = np.diag(np.dot(W.T,W))
	idx_e = []
	params_e = []
	params_r = []
	for i in range(col_norm.shape[0]):
		if col_norm[i] < tol_e: 
			idx_e.append(i)
			params_e.append(list(params_std.keys())[i])
		else: 
			params_r.append(list(params_std.keys())[i])
	W_e = np.delete(W, idx_e, 1)
	return W_e, params_e, params_r

#QR decompostion, rank revealing
def QR_pivoting(tau, W_e, params_r):
	"""This function calculates QR decompostion with pivoting, finds rank of regressor,
	and calculates base parameters
		Input: 	W_e: reduced regressor
				params_r: inertial parameters corresponding to W_e 
		Output: W_b: base regressor
				phi_b: values of base parameters
				numrank_W: numerical rank of regressor, determined by using a therehold
				params_rsorted: inertial parameters included in base parameters """
	
	Q, R, P = linalg.qr(W_e, pivoting = True) #scipy has QR pivoting
	# print(pd.DataFrame(R[0:7,:]).to_latex())
	# sort params as decreasing order of diagonal of R 
	params_rsorted = []
	for i in range(P.shape[0]): 
		print(i, ": ", params_r[P[i]], "---",abs(np.diag(R)[i]))
		params_rsorted.append(params_r[P[i]])
	
	#find rank of regressor
	numrank_W = 0
	epsilon = np.finfo(float).eps# machine epsilon
	# tolpal = W_e.shape[0]*abs(np.diag(R).max())*epsilon#rank revealing tolerance
	tolpal = 0.02
	for i in range(np.diag(R).shape[0]):
		if abs(np.diag(R)[i]) > tolpal:
			# print(abs(np.diag(R)[i]))
			continue
		else:
			# print(abs(np.diag(R)[i]))
			numrank_W = i
			break

	#regrouping, calculating base params, base regressor
	print("rank of base regressor: ", numrank_W)
	R1 = R[0:numrank_W,0:numrank_W]
	Q1 = Q[:,0:numrank_W]
	R2 = R[0:numrank_W,numrank_W:R.shape[1]]
	beta = np.round(np.dot(np.linalg.inv(R1),R2),6)#regrouping coefficient

	phi_b = np.round(np.dot(np.linalg.inv(R1),np.dot(Q1.T,tau)),6)#values of base params
	W_b = np.dot(Q1,R1)#base regressor
	params_base = params_rsorted[:numrank_W]
	params_rgp = params_rsorted[numrank_W:]
	print('regrouped params: ',params_rgp)
	for i in range(numrank_W):
		for j in range(beta.shape[1]):
			if beta[i,j] == 0:	
				params_base[i] = params_base[i]
			elif beta[i,j] < 0:
				params_base[i] = params_base[i] + ' - '+str(abs(beta[i,j])) + '*'+ str(params_rgp[j])
			else:
				params_base[i] = params_base[i] + ' + '+str(abs(beta[i,j])) + '*'+ str(params_rgp[j])
	
	# print('base parameters and their identified values: ')
	base_parameters = dict(zip(params_base,phi_b))
	# table = [params_base, phi_b]
	# print(tabulate(table))
	return W_b, base_parameters

#display 
# If you want to visualize the robot in this example,
# you can choose which visualizer to employ
# by specifying an option from the command line:
# GepettoVisualizer: -g
# MeshcatVisualizer: -m

def visualization(robot): 
	VISUALIZER = None
	if len(argv)>1:
		opt = argv[1]
		if opt == '-g':
			VISUALIZER = GepettoVisualizer
		elif opt == '-m':
			VISUALIZER = MeshcatVisualizer
		#else:
		#    raise ValueError("Unrecognized option: " + opt)
	# dt = 1e-2
	if VISUALIZER:
		robot.setVisualizer(VISUALIZER())
		robot.initViewer()
		robot.loadViewerModel("pinocchio")
		robot.display(robot.q0)
		# for k in range(q.shape[0]):
		# 	t0 = time.time()
		# 	robot.display(q[k,:])
		# 	t1 = time.time()
		# 	elapsed_time = t1 - t0
		# 	if elapsed_time < dt:
		# 		time.sleep(dt - elapsed_time)



def main():
	params_std = standardParameters(njoints,fv,fs,Ia,off, Iam6, fvm6, fsm6)
	table_stdparams = pd.DataFrame(params_std.items(), columns = ["Standard Parameters", "Value"])
	print(table_stdparams.to_latex())#latex table
	print("###########################")
	if not isFext:
		q , qd, qdd = generateWaypoints(N, nq, nv, -1, 1)
		tau, W = iden_model(model, data, N,  nq, nv,njoints, q, qd, qdd)
	else:
		q, qd, qdd = generateWaypoints_fext(N, robot, -1, 1)
		tau, W = iden_model_fext(model, data, N,  nq, nv,njoints, q, qd, qdd)
	W_e, params_r = eliminateNonAffecting(W, 1e-6)
	W_b, base_parameters  = QR_pivoting(W_e, params_r)
	table_base = pd.DataFrame(base_parameters.items(), columns = ["Base Parameters", "Value"])
	print(table_base.to_latex())#latex
	print("###########################")
	print('condition number of base regressor: ',np.linalg.cond(W_b))
	U, S, VT = np.linalg.svd(W_b)
	print('singular values of base regressor:', S)
	visualization(robot)
	# print(nq, nv)

isFext = False
isActuator_int = True
isFrictionincld = True
isOffset = True
isCoupling =  True
if len(argv)>1:
	if argv[1] == '-f':
		isFrictionincld = True


fv = np.array([8.05e0,5.53e0,1.97e0,1.11e0,1.86e0,6.5e-1])
fs = np.array([7.14e0,8.26e0,6.34e0,2.48e0,3.03e0,2.82e-1])
Ia = np.array([3.62e-1,3.62e-1,9.88e-2,3.13e-2,4.68e-2,1.05e-2])
off = np.array([3.92e-1,1.37e0,3.26e-1,-1.02e-1,-2.88e-2,1.27e-1])
Iam6 = 9.64e-3
fvm6 = 6.16e-1
fsm6 = 1.95e0

#load robot
robot = loadModels("staubli_tx40_description", "tx40_mdh.urdf")
# robot = loadModels("2DOF_description", "2DOF_description.urdf")
# robot = loadModels("SC_3DOF", "3DOF.urdf")
model = robot.model
print(model)
data = robot.data	
nq, nv , njoints = model.nq, model.nv, model.njoints


if __name__ == '__main__':
	params_std = standardParameters(njoints,fv,fs,Ia,off,Iam6,fvm6,fsm6)
	table_stdparams = pd.DataFrame(params_std.items(), columns = ["Standard Parameters", "Value"])
	print(table_stdparams)

	# print("identification on exp data")
	dt = 1/5000
	curr_data = pd.read_csv('src/thanh/curr_data.csv').to_numpy()
	pos_data = pd.read_csv('src/thanh/pos_read_data.csv').to_numpy()
	#Nyquist freq/0.5*sampling rate fs = 0.5 *5 kHz

	N = pos_data.shape[0]
	y = np.zeros([N,curr_data.shape[1]])
	t = np.linspace(0,N-1,N)*dt
	q = np.zeros([N,pos_data.shape[1]])

	y = curr_data
	q = pos_data
	
	N1 = 32
	N2 = 32
	N3 = 45
	N4 = -48
	N5 = 45
	N6 = 32

	red_diag= np.diag([N1,N2,N3,N4,N5,N6])

	#calculate joint position = inv(reduction ration matrix)*motor_encoder_angle
	red_q = red_diag
	red_q[5,4] = N6
	q_T = np.dot(np.linalg.inv(red_q),q.T)
	q = q_T.T
	q[:,1] += -np.pi/2
	q[:,2] += np.pi/2
	q[:,5] += np.pi

	b,a = signal.butter(4,100/2500, 'low')
	for j in range(q.shape[1]):
		q[:,j] = signal.filtfilt(b,a,q[:,j])
	#calculate vel and acc by derivating data q
	dq = np.zeros([q.shape[0],q.shape[1]])
	ddq = np.zeros([q.shape[0],q.shape[1]])
	for i in range(pos_data.shape[1]):
		dq[:,i] = np.gradient(q[:,i], edge_order = 2)/dt
		ddq[:,i] = np.gradient(dq[:,i],edge_order = 2)/dt
		# plt.plot(t, q[:,i])
		# plt.plot(t, dq[:,i])
		# plt.plot(t, ddq[:,i])



	#build identification model
	qd = dq
	qdd = ddq

	W= build_regressor_full(model, data, N,  nq, nv,njoints, q, qd, qdd)
	# print(W.shape)
	W = add_coupling(W,model, data, N,  nq, nv,njoints, q, qd, qdd)
	print(W.shape)

	#calculate joint torques from measured current
	#reduction gear ratio matrix*motor_torques
	red_tau = red_diag
	red_tau[4,5] = N6
	# print(y.shape)
	tau_T = np.dot(red_tau,y.T)
	# print(tau_T.shape)
	#efficiency = torque_out/torque_in
	# rend = np.diag([0.75,0.75,0.5,0.5,0.5,0.63]) 
	# tau_T = np.dot(rend,tau_T)

	tau_data = np.asarray(tau_T).ravel()#straight a matrix to a vector

	#simulate joint torques from q, dq, ddq data
	# tau_cal = get_torque_rand(model, data, N,  nq, nv,njoints, q, qd, qdd, Ia, off, Iam6, fvm6, fsm6)
	# mse_tau = np.square(tau_data - tau_cal)
	# error_tau = np.mean(tau_data != tau_cal)
	# print("residual error between calculated and measured tau: ", mse_tau)
	# print("percentage error between calculated and measured tau: ", error_tau)

	tau = tau_data
	# tau = tau_cal

	#low pass butter filtering on tau and columns of W
	# b,a = signal.butter(4,100/2500, 'low')
	
	#chebyshev I lowpass
	b,a = signal.cheby1(100,5,100,'low',fs=5000, output='ba')

	#forward-backward filter to prevent phase shift
	for j in range(W.shape[1]):
		W[:,j] = signal.filtfilt(b,a,W[:,j])
	tau = signal.filtfilt(b,a,tau)


	t = np.linspace(0,1,tau.shape[0])
	plt.figure()
	plt.plot(t,tau_data)
	plt.plot(t,tau)
	plt.show()
	#downsampling and identification on exp data
	
	N_ = W.shape[0]//100
	W_ = np.zeros([N_,W.shape[1]])
	tau_ = np.zeros(N_)
	for i in range(N_):
			# print(i)
		W_[i,:] = W[i*100,:]
		tau_[i] = tau[i*100]
	# tau_ = signal.decimate(tau, 5, n=None, zero_phase=True)
	# tau_ = signal.decimate(tau_, 9, n=None, zero_phase=True)

	# for i in range(W.shape[1]):
	# 	Wtemp = signal.decimate(W[:,i], 5, n=10, zero_phase=True)
	# 	W_[:,i] = signal.decimate(Wtemp, 9, n=10, zero_phase=True)

	print(W_.shape)
	#elimate and QR decomposition
	W_e, params_e, params_r = eliminateNonAffecting(W_, 0.001)
	W_b, base_parameters  = QR_pivoting(tau_, W_e, params_r)

	#identification on random data points
	# print("identification on random data")
	# N_ = 1000
	# q_rand , qd_rand, qdd_rand = generateWaypoints(N_, nq, nv, -1, 1)
	# tau_rand= get_torque_rand(model, data, N_,  nq, nv, njoints, q_rand, qd_rand, qdd_rand, Ia, off, Iam6, fvm6, fsm6)
	# W_rand	= build_regressor_full(model, data, N_,  nq, nv,njoints, q_rand, qd_rand, qdd_rand)
	# W_rand 	= add_coupling(W_rand,model, data, N_,  nq, nv,njoints, q_rand, qd_rand, qdd_rand)
	# # print('W_rand',W_rand[[-4,-3,-2,-1],:])
	# # print('W_rand size',W_rand.shape)
	# W_e, params_e, params_r = eliminateNonAffecting(W_rand, 0.001)
	# W_b, base_parameters  = QR_pivoting(tau_rand, W_e, params_r)


	print("eleminanted parameters: ",params_e)
	print('condition number of base regressor: ',np.linalg.cond(W_b))
	table_base = pd.DataFrame(base_parameters.items(), columns = ["Base Parameters", "Value"])
	path_save = join(dirname(dirname(str(abspath(__file__)))),"identification/src/thanh/TX40_bp_mdh.csv")
	with open(path_save, "w") as output_file:
		w = csv.writer(output_file)
		for key, val in base_parameters.items():
			w.writerow([key, val])

	