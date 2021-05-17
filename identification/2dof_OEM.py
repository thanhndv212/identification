import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper
from pinocchio.visualize import (GepettoVisualizer, MeshcatVisualizer)
from pinocchio.utils import *

from sys import argv
import os
from os.path import dirname, join, abspath

import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math
import time
from ndcurves import (polynomial, piecewise, cubic_hermite_spline)
from trajectories import trapezoidal
import eigenpy
import cyipopt

from load_models import *
from regressor_TX40 import(standardParameters_modified,build_regressor_basic_modified, eliminateNonAffecting, double_QR)

def get_torque_rand(model, data, N, nq, nv, njoints, q, v, a):
    tau = np.zeros(nv * N)
    for i in range(N):
        for j in range(nv):
            tau[j * N + i] = pin.rnea(model, data, q[i, :], v[i, :], a[i, :])[j] 
    return tau


def objective_func(N, f, Xp):


    #create time_points
    time_points = np.array([[s*0.5] for s in range(N)])

    #randomize a set of waypoints
    points = np.reshape(Xp, (2,N))
    points_derivative = 5*np.array(np.random.uniform(-math.pi/2, math.pi/2, size=(2,N)))
    #generate splines
    polC0 = piecewise.FromPointsList( points, points_derivative, time_points)

    #caculate q, dq, ddq
    t = np.array([[(i / f * (polC0.max() - polC0.min()) + polC0.min())] for i in range(f+1)])

    q = np.array([polC0(t[i,0]) for i in range(f+1)])
    dq = np.array([polC0.derivate(t[i,0],1) for i in range(f+1)], dtype = 'float')
    ddq = np.array([polC0.derivate(t[i,0],2) for i in range(f+1)], dtype = 'float')

    #visualize motion
    robot = loadModels("2DOF_description", "2DOF_description.urdf")
    visualization(robot, q)

    #build base regressor
    model, data = robot.model, robot.data
    nq, nv,  njoints = model.nq, model.nv, model.njoints

    params_std = standardParameters_modified(model, njoints)
    W = build_regressor_basic_modified(model, data, f+1, nq, nv, njoints, q, dq, ddq)
    W_e, params_e, params_r = eliminateNonAffecting(W, params_std, tol_e = 1e-10)
    tau = get_torque_rand(model, data, f+1, nq, nv, njoints, q, dq, ddq)
    W_b, base_parameters, params_base, phi_b = double_QR(tau, W_e, params_r)

    ############IpOpt

    #objective funtcion
    f = np.linalg.cond(W_b)
    return f


class cond_Wb():
    def objective(self, Xp):
        return objective_func(Xp)
    def intermediate(
            self,
            alg_mod,
            iter_count,
            obj_value,
            inf_pr,
            inf_du,
            mu,
            d_norm,
            regularization_size,
            alpha_du,
            alpha_pr,
            ls_trials
            ):
        #
        # Example for the use of the intermediate callback.
        #
        print("Objective value at iteration #%d is - %g" % (iter_count, obj_value))

N = 11
f = 1000
Xp0 = np.random.uniform(-math.pi/2, math.pi/2, size=(2*N,)).tolist()
objective_func(N, f, Xp0)
# lb = [-math.pi]*(2*N)
# ub = [math.pi]*(2*N)
# cl = []
# cu = []
# nlp = cyipopt.problem(
#             n=len(Xp0),
#             m=len(cl),
#             problem_obj=cond_Wb(),
#             lb=lb,
#             ub=ub,
#             cl=cl,
#             cu=cu
#             )
# nlp.addOption('mu_strategy', 'adaptive')
# nlp.addOption('tol', 1e-7)
# X_opt, infor = nlp.solve()
#boundaries

#solve