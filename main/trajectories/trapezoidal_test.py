import numpy as np 
from matplotlib import pyplot as plt
import time
nsamples = 1000

ts = 0.01

nwaypoints = 4 #number of joint config including start and end

run_time = nsamples*ts #total time

njoints = 2 #number of joints

Kq = np.empty([njoints,2]) #lower bound and upper bounds of joints limits

Kv = np.empty([njoints,1]) #velocity limits

Ka = np.empty([njoints,1]) #acceleration limits

#choose sub runtime tf 
tf = np.empty([nwaypoints-1,1])
#choose velocity  profile 
acc = [0.2, -0.2, 0.2 ] #absolute values of first duration's acc

delta_t1 = [100, 200, 300]# first duration

delta_t2 = [500, 400, 300]#second duration

delta_t3 = [] # tf - delta_t1 - delta_t2

def trapTraj_2P(a1,q0,n1, n2, N, ts):
	#initial
	q = np.array([q0])
	qd = np.array([0])
	qdd = np.array([a1])

	a3 = -a1*n1/(N-n1-n2)

	for i in range(1, N):
		if i < n1: 
			qdd = np.append(qdd, a1)
			qd = np.append(qd,qd[i-1] + qdd[i-1]*ts)
			q =np.append(q,q[i-1] + qd[i-1]*ts)
		elif i >= n1 and i< (n1 + n2):
			qdd = np.append(qdd,0)
			qd = np.append(qd, qd[i-1] + qdd[i-1]*ts)
			q = np.append(q,q[i-1] + qd[i-1]*ts)
		else: 
			qdd = np.append(qdd,a3)
			qd = np.append(qd,qd[i-1] + qdd[i-1]*ts)
			q =np.append(q,q[i-1] + qd[i-1]*ts)
	return q, qd, qdd
q = np.array([0])
qd = np.array([0])
qdd = np.array([acc[0]])
for k in range(nwaypoints-1):
	q_, qd_, qdd_ = trapTraj_2P(acc[k], q[-1], delta_t1[k], delta_t2[k], nsamples, ts)
	q = np.append(q,q_)
	qd = np.append(qd,qd_)
	qdd = np.append(qdd,qdd_)	

def plotTraj():
	time_slot = np.linspace(0.,((nwaypoints-1)*nsamples +1)*ts, num = ((nwaypoints-1)*nsamples +1))
	print(time_slot[-1])
	fig, axs = plt.subplots(3,1)
	axs[0].plot(time_slot,q)
	axs[1].plot(time_slot,qd)
	axs[2].plot(time_slot,qdd)
	plt.show()

plotTraj()