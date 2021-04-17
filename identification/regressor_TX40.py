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
def standardParameters(njoints,fv,fc,Ia,off):
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
			phi.extend([fv,fs])
			params.extend(['fv' + str(k),'fs' + str(k)])
	if isActuator_int:
		for  q in range(1,njoints):
			phi.extend([Ia[i-1]])
			params.extend(['Ia' + str(q)])
	if isOffset:
		for  q in range(1,njoints):
			phi.extend([off[i-1]])
			params.extend(['off' + str(q)])
	if isCoupling:
		phi.extend([0.5,0.6,0.4])
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
			if j == (nv-1):
				W[j*N + i, -3] = a[i,nv-2]
				W[j*N + i, -2] = v[i,nv-2]
				W[j*N + i, -1] = np.sign(v[i,nv-2] + v[i,nv-1])			
			W[j*N + i, 10*(njoints-1)+2*j + 1] = np.sign(v[i,j])	
	return W
def get_torque_rand(model, data, N,  nq, nv,njoints, q, v, a, Ia, off, Iam6, fvm6, fsm6):
	tau = np.zeros(nv*N)
	for i in range(N):
		for j in range(nv):
			tau[j*N + i] = pin.rnea(model, data, q[i,:], v[i,:], a[i,:])[j] + v[i,j]*fv + np.sign(v[i,j])*fs + Ia[j]*a[i,j] + off[j]
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
				tau[j*N + i] = tau[j*N + i] + v[i,j]*fv + np.sign(v[i,j])*fs
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
	# print(col_norm)
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
def QR_pivoting(W_e, params_r):
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
	for ind in P: 
		params_rsorted.append(params_r[ind])
	#find rank of regressor
	numrank_W = 0
	epsilon = np.finfo(float).eps# machine epsilon
	tolpal = W_e.shape[0]*abs(np.diag(R).max())*epsilon#rank revealing tolerance
	for i in range(np.diag(R).shape[0]):
		if abs(np.diag(R)[i]) > tolpal:
			continue
		else: 
			numrank_W = i
			break
	#regrouping, calculating base params, base regressor
	print(numrank_W)
	R1 = R[0:numrank_W,0:numrank_W]
	Q1 = Q[:,0:numrank_W]
	R2 = R[0:numrank_W,numrank_W:R.shape[1]]
	beta = np.round(np.dot(np.linalg.inv(R1),R2),3)#regrouping coefficient

	phi_b = np.round(np.dot(np.linalg.inv(R1),np.dot(Q1.T,tau)),3)#values of base params
	W_b = np.dot(Q1,R1)#base regressor
	params_base = params_rsorted[:numrank_W]
	params_rgp = params_rsorted[numrank_W:]
	print(params_rgp)
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

# print('idpnd. parameters: ', params_idp)
# print('regrouped parameters: ', params_rgp)


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

isFext = False
isActuator_int = True
isFrictionincld = True
isOffset = True
isCoupling =  True
if len(argv)>1:
	if argv[1] == '-f':
		isFrictionincld = True
fv = 0.05
fs = 0.01


robot = loadModels("staubli_tx40_description", "tx40.urdf")
# robot = loadModels("2DOF_description", "2DOF_description.urdf")
# robot = loadModels("SC_3DOF", "3DOF.urdf")
model = robot.model
print(model)
data = robot.data	
nq, nv , njoints = model.nq, model.nv, model.njoints
Ia = np.array([1,1,1,1,1,1])
off = np.array([1,1,1,1,1,1])
Iam6 = 0.5
fvm6 = 0.6 
fsm6 = 0.4
#numbers of samples
# N = 1000

def main():
	params_std = standardParameters(njoints,fv,fc,Ia)
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
if __name__ == '__main__':
	params_std = standardParameters(njoints,fv,fs,Ia,off)
	table_stdparams = pd.DataFrame(params_std.items(), columns = ["Standard Parameters", "Value"])
	print(table_stdparams)

	# q , qd, qdd = generateWaypoints(N, nq, nv, -2, 2)


	N1 = 32
	N2 = 32
	N3 = 45
	N4 = -48
	N5 = 45
	N6 = 32

	red= np.diag([N1,N2,N3,N4,N5,N6])
	red[4,5] = N6


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

	tau_T = np.dot(red,y.T)
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

	N = q.shape[0]
	qd = dq
	qdd = ddq

	W= build_regressor_full(model, data, N,  nq, nv,njoints, q, qd, qdd)
	print(W.shape)
	W = add_coupling(W,model, data, N,  nq, nv,njoints, q, qd, qdd)
	print(W.shape)
	# tau = get_torque_rand(model, data, N,  nq, nv,njoints, q, qd, qdd, Ia, off, Iam6, fvm6, fsm6)


	W_e, params_e, params_r = eliminateNonAffecting(W, 1e-6)
	# print(W_e)
	# print(params_e)
	W_b, base_parameters  = QR_pivoting(W_e, params_r)
	print('condition number of base regressor: ',np.linalg.cond(W_b))
	table_base = pd.DataFrame(base_parameters.items(), columns = ["Base Parameters", "Value"])
	
	path_save = join(dirname(dirname(str(abspath(__file__)))),"identification/src/thanh/TX40_bp.text")
	with open(path_save, "w") as output_file:
		# table_base.to_string(output_file)
		print(base_parameters, file = output_file)
	print(table_base)
	