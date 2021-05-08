import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper
from pinocchio.visualize import (GepettoVisualizer, MeshcatVisualizer)
from sys import argv
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
from settings import *



def standardParameters(njoints, fv, fs, Ia, off, Iam6, fvm6, fsm6):
    """This function prints out the standard inertial parameters obtained from 3D design.
            Note: a flag IsFrictioncld to include in standard parameters
            Input:  njoints: number of joints
            Output: params_std: a dictionary of parameter names and their values"""
    params_name = ['m', 'mx', 'my', 'mz', 'Ixx',
                   'Ixy', 'Iyy', 'Ixz', 'Iyz', 'Izz']
    phi = []
    params = []
    for i in range(1, njoints):
        P = model.inertias[i].toDynamicParameters()
        for k in P:
            phi.append(k)
        for j in params_name:
            if not isFext:
                params.append(j + str(i))
            else:
                params.append(j + str(i - 1))
    if isFrictionincld:
        for k in range(1, njoints):
            phi.extend([fv[k - 1], fs[k - 1]])
            params.extend(['fv' + str(k), 'fs' + str(k)])
    if isActuator_int:
        for k in range(1, njoints):
            phi.extend([Ia[k - 1]])
            params.extend(['Ia' + str(k)])
    if isOffset:
        for k in range(1, njoints):
            phi.extend([off[k - 1]])
            params.extend(['off' + str(k)])
    if isCoupling:
        phi.extend([Iam6, fvm6, fsm6])
        params.extend(['Iam6', 'fvm6', 'fsm6'])
    params_std = dict(zip(params, phi))
    return params_std


def standardParameters_modified(njoints, fv, fs, Ia, off, Iam6, fvm6, fsm6):
    """This function prints out the standard inertial parameters obtained from 3D design.
            Note: a flag IsFrictioncld to include in standard parameters
            Input:  njoints: number of joints
            Output: params_std: a dictionary of parameter names and their values"""
    params_name = ['Ixx', 'Ixy', 'Ixz', 'Iyy',
                   'Iyz', 'Izz', 'mx', 'my', 'mz', 'm']
    phi = []
    params = []
    # TODO: change order of values in phi['m', 'mx','my','mz','Ixx','Ixy','Iyy','Ixz', 'Iyz','Izz']
    # corresponding to params_name ['Ixx','Ixy','Ixz','Iyy','Iyz','Izz','mx','my','mz','m']
    for i in range(1, njoints):
        P = model.inertias[i].toDynamicParameters()
        P_mod = np.zeros(P.shape[0])
        P_mod[9] = P[0]  # m
        P_mod[8] = P[3]  # mz
        P_mod[7] = P[2]  # my
        P_mod[6] = P[1]  # mx
        P_mod[5] = P[9]  # Izz
        P_mod[4] = P[8]  # Iyz
        P_mod[3] = P[6]  # Iyy
        P_mod[2] = P[7]  # Ixz
        P_mod[1] = P[5]  # Ixy
        P_mod[0] = P[4]  # Ixx
        for k in P_mod:
            phi.append(k)
        phi.extend([Ia[i - 1]])
        phi.extend([fv[i - 1], fs[i - 1]])
        phi.extend([off[i - 1]])
        for j in params_name:
            if not isFext:
                params.append(j + str(i))
            else:
                params.append(j + str(i - 1))
        params.extend(['Ia' + str(i)])
        params.extend(['fv' + str(i), 'fs' + str(i)])
        params.extend(['off' + str(i)])

    if isCoupling:
        phi.extend([Iam6, fvm6, fsm6])
        params.extend(['Iam6', 'fvm6', 'fsm6'])
    params_std = dict(zip(params, phi))
    return params_std


def generateWaypoints(N, nq, nv, mlow, mhigh):
    """This function generates N random values for joints' position,velocity, acceleration.
            Input:  N: number of samples
                            nq: length of q, nv : length of v
                            mlow and mhigh: the bound for random function
            Output: q, v, a: joint's position, velocity, acceleration"""
    q = np.empty((1, nq))
    v = np.empty((1, nv))
    a = np.empty((1, nv))
    for i in range(N):
        q = np.vstack((q, np.random.uniform(low=mlow, high=mhigh, size=(nq,))))
        v = np.vstack((v, np.random.uniform(low=mlow, high=mhigh, size=(nv,))))
        a = np.vstack((a, np.random.uniform(low=mlow, high=mhigh, size=(nv,))))
    return q, v, a


def generateWaypoints_fext(N, robot, mlow, mhigh):
    """This function generates N random values for joints' position,velocity, acceleration.
            Input:  N: number of samples
                            nq: length of q, nv : length of v
                            mlow and mhigh: the bound for random function
            Output: q, v, a: joint's position, velocity, acceleration"""
    nq = robot.model.nq
    nv = robot.model.nv
    q0 = robot.q0[:7]
    v0 = np.zeros(6)
    a0 = np.zeros(6)
    q = np.empty((1, nq))
    v = np.empty((1, nv))
    a = np.empty((1, nv))
    for i in range(N):

        q_ = np.append(q0, np.random.uniform(
            low=mlow, high=mhigh, size=(nq - 7,)))
        q = np.vstack((q, q_))

        v_ = np.append(v0, np.random.uniform(
            low=mlow, high=mhigh, size=(nv - 6,)))
        v = np.vstack((v, v_))

        a_ = np.append(a0, np.random.uniform(
            low=mlow, high=mhigh, size=(nv - 6,)))
        a = np.vstack((a, a_))
    return q, v, a


def build_regressor_basic(model, data, N, nq, nv, njoints, q, v, a):
    W = np.zeros([N * nv, 10 * nv])
    for i in range(N):
        W_temp = pin.computeJointTorqueRegressor(
            model, data, q[i, :], v[i, :], a[i, :])
        for j in range(W_temp.shape[0]):
            W[j * N + i, :] = W_temp[j, :]
    return W


def build_regressor_basic_modified(model, data, N, nq, nv, njoints, q, v, a):
    # TODO: reorgnize columns from ['m', 'mx','my','mz','Ixx','Ixy','Iyy','Ixz', 'Iyz','Izz']
    # to ['Ixx','Ixy','Ixz','Iyy','Iyz','Izz','mx','my','mz','m']
    W = np.zeros([N * nv, 10 * nv])
    W_mod = np.zeros([N * nv, 10 * nv])
    for i in range(N):
        W_temp = pin.computeJointTorqueRegressor(
            model, data, q[i, :], v[i, :], a[i, :])
        for j in range(W_temp.shape[0]):
            W[j * N + i, :] = W_temp[j, :]

    for k in range(nv):
        W_mod[:, 10 * k + 9] = W[:, 10 * k + 0]  # m
        W_mod[:, 10 * k + 8] = W[:, 10 * k + 3]  # mz
        W_mod[:, 10 * k + 7] = W[:, 10 * k + 2]  # my
        W_mod[:, 10 * k + 6] = W[:, 10 * k + 1]  # mx
        W_mod[:, 10 * k + 5] = W[:, 10 * k + 9]  # Izz
        W_mod[:, 10 * k + 4] = W[:, 10 * k + 8]  # Iyz
        W_mod[:, 10 * k + 3] = W[:, 10 * k + 6]  # Iyy
        W_mod[:, 10 * k + 2] = W[:, 10 * k + 7]  # Ixz
        W_mod[:, 10 * k + 1] = W[:, 10 * k + 5]  # Ixy
        W_mod[:, 10 * k + 0] = W[:, 10 * k + 4]  # Ixx
    return W_mod


def add_friction(W, model, data, N, nq, nv, njoints, q, v, a):
    # TODO: break the model modular
    W = np.c_[W, np.zeros([W.shape[0], 2 * nv])]
    for i in range(N):
        for j in range(nv):
            W[j * N + i, W.shape[1] + 2 * j] = v[i, j]
            W[j * N + i, W.shape[1] + 2 * j + 1] = np.sign(v[i, j])
    pass


def build_regressor_w_friction(model, data, N, nq, nv, njoints, q, v, a):
    W = np.zeros([N * nv, 10 * nv + 2 * nv])
    for i in range(N):
        W_temp = pin.computeJointTorqueRegressor(
            model, data, q[i, :], v[i, :], a[i, :])
        for j in range(W_temp.shape[0]):
            W[j * N + i, 0:10 * nv] = W_temp[j, :]
            W[j * N + i, 10 * nv + 2 * j] = v[i, j]
            W[j * N + i, 10 * nv + 2 * j + 1] = np.sign(v[i, j])
    return W


def add_motor_inertia(W, model, data, N, nq, nv, njoints, q, v, a):
    # TODO: break the model modular
    W = np.c_[W, np.zeros([W.shape[0], 2 * nv])]
    for i in range(N):
        for j in range(nv):
            W[j * N + i, W.shape[1] + j] = a[i, j]
            W[j * N + i, W.shape[1] + nv + j] = 1


def build_regressor_full(model, data, N, nq, nv, njoints, q, v, a):
    W = np.zeros([N * nv, 10 * nv + 2 * nv + nv + nv])

    for i in range(N):
        W_temp = pin.computeJointTorqueRegressor(
            model, data, q[i, :], v[i, :], a[i, :])
        for j in range(W_temp.shape[0]):
            W[j * N + i, 0:10 * nv] = W_temp[j, :]
            W[j * N + i, 10 * nv + 2 * j] = v[i, j]
            W[j * N + i, 10 * nv + 2 * j + 1] = np.sign(v[i, j])
            W[j * N + i, 10 * nv + 2 * nv + j] = a[i, j]
            W[j * N + i, 10 * nv + 2 * nv + nv + j] = 1
    return W


def build_regressor_full_modified(model, data, N, nq, nv, njoints, q, v, a):
    W = np.zeros([N * nv, 10 * nv + 2 * nv + nv + nv])

    for i in range(N):
        W_temp = pin.computeJointTorqueRegressor(
            model, data, q[i, :], v[i, :], a[i, :])
        for j in range(W_temp.shape[0]):
            W[j * N + i, 0:10 * nv] = W_temp[j, :]
            W[j * N + i, 10 * nv + 2 * j] = v[i, j]  # fv
            W[j * N + i, 10 * nv + 2 * j + 1] = np.sign(v[i, j])  # fs
            W[j * N + i, 10 * nv + 2 * nv + j] = a[i, j]  # ia
            W[j * N + i, 10 * nv + 2 * nv + nv + j] = 1  # off
    W_mod = np.zeros([N * nv, 10 * nv + 2 * nv + nv + nv])
    for k in range(nv):
        W_mod[:, 14 * k + 10] = W[:, 10 * nv + 2 * nv + k]  # ia
        W_mod[:, 14 * k + 11] = W[:, 10 * nv + 2 * k]  # fv
        W_mod[:, 14 * k + 12] = W[:, 10 * nv + 2 * k + 1]  # fs
        W_mod[:, 14 * k + 13] = W[:, 10 * nv + 2 * nv + nv + k]  # off
        W_mod[:, 14 * k + 9] = W[:, 10 * k + 0]  # m
        W_mod[:, 14 * k + 8] = W[:, 10 * k + 3]  # mz
        W_mod[:, 14 * k + 7] = W[:, 10 * k + 2]  # my
        W_mod[:, 14 * k + 6] = W[:, 10 * k + 1]  # mx
        W_mod[:, 14 * k + 5] = W[:, 10 * k + 9]  # Izz
        W_mod[:, 14 * k + 4] = W[:, 10 * k + 8]  # Iyz
        W_mod[:, 14 * k + 3] = W[:, 10 * k + 6]  # Iyy
        W_mod[:, 14 * k + 2] = W[:, 10 * k + 7]  # Ixz
        W_mod[:, 14 * k + 1] = W[:, 10 * k + 5]  # Ixy
        W_mod[:, 14 * k + 0] = W[:, 10 * k + 4]  # Ixx
    return W_mod


def add_coupling(W, model, data, N, nq, nv, njoints, q, v, a):
    W = np.c_[W, np.zeros([W.shape[0], 3])]
    for i in range(N):
        # joint 5
        W[4 * N + i, W.shape[1] - 3] = a[i, 5]
        W[4 * N + i, W.shape[1] - 2] = v[i, 5]
        W[4 * N + i, W.shape[1] - 1] = np.sign(v[i, 4] + v[i, 5])
    # joint 6
        W[5 * N + i, W.shape[1] - 3] = a[i, 4]
        W[5 * N + i, W.shape[1] - 2] = v[i, 4]
        W[5 * N + i, W.shape[1] - 1] = np.sign(v[i, 4] + v[i, 5])

    return W


def get_torque_rand(model, data, N, nq, nv, njoints, q, v, a, Ia, off, Iam6, fvm6, fsm6):
    tau = np.zeros(nv * N)
    for i in range(N):
        for j in range(nv):
            tau[j * N + i] = pin.rnea(model, data, q[i, :], v[i, :], a[i, :])[j] + \
                v[i, j] * fv[j] + np.sign(v[i, j]) * fs[j] + Ia[j] * a[i, j] + off[j]
            if j == nv - 2:
                tau[j * N + i] = tau[j * N + i] + Iam6 * v[i, nv - 1] + fvm6 * \
                    v[i, nv - 1] + fsm6 * np.sign(v[i, nv - 2] + v[i, nv - 1])
            if j == nv - 1:
                tau[j * N + i] = tau[j * N + i] + Iam6 * v[i, nv - 2] + fvm6 * \
                    v[i, nv - 2] + fsm6 * np.sign(v[i, nv - 2] + v[i, nv - 1])
    return tau


def iden_model(model, data, N, nq, nv, njoints, q, v, a):
    """This function calculates joint torques and generates the joint torque regressor.
            Note: a flag IsFrictionincld to include in dynamic model
            Input:  model, data: model and data structure of robot from Pinocchio
                            q, v, a: joint's position, velocity, acceleration
                            N : number of samples
                            nq: length of q
            Output: tau: vector of joint torque
                            W : joint torque regressor"""
    tau = np.empty(nv * N)
    W = np.empty([N * nv, 10 * nv])
    for i in range(N):
        tau_temp = pin.rnea(model, data, q[i, :], v[i, :], a[i, :])
        W_temp = pin.computeJointTorqueRegressor(
            model, data, q[i, :], v[i, :], a[i, :])
        for j in range(W_temp.shape[0]):
            tau[j * N + i] = tau_temp[j]
            W[j * N + i, :] = W_temp[j, :]
    if isFrictionincld:
        W = np.c_[W, np.zeros([N * nv, 2 * nv])]
        for i in range(N):
            for j in range(nv):
                tau[j * N + i] = tau[j * N + i] + v[i, j] * \
                    fv[j] + np.sign(v[i, j]) * fs[j]
                W[j * N + i, 10 * nv + 2 * j] = v[i, j]
                W[j * N + i, 10 * nv + 2 * j + 1] = np.sign(v[i, j])
    return tau, W


def iden_model_fext(model, data, N, nq, nv, njoints, q, v, a):
    """This function calculates joint torques and generates the joint torque regressor.
            Note: a flag IsFrictioncld to include in dynamic model
            Input:  model, data: model and data structure of robot from Pinocchio
                            q, v, a: joint's position, velocity, acceleration
                            N : number of samples
                            nq: length of q
            Output: tau: vector of joint torque
                            W : joint torque regressor"""
    tau = np.empty(6 * N)
    W = np.empty([N * 6, 10 * nv])
    for i in range(N):
        tau_temp = pin.rnea(model, data, q[i, :], v[i, :], a[i, :])
        W_temp = pin.computeJointTorqueRegressor(
            model, data, q[i, :], v[i, :], a[i, :])
        for j in range(6):
            tau[j * N + i] = tau_temp[j]
            W[j * N + i, :] = W_temp[j, :]
    return tau, W


def eliminateNonAffecting(W, tol_e):
    """This function eliminates columns which has L2 norm smaller than tolerance.
            Input:  W: joint torque regressor
                            tol_e: tolerance 
            Output: W_e: reduced regressor
                            params_r: corresponding parameters to columns of reduced regressor"""
    col_norm = np.diag(np.dot(W.T, W))
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


def QR_pivoting(tau, W_e, params_r):
    """This function calculates QR decompostion with pivoting, finds rank of regressor,
    and calculates base parameters
            Input:  W_e: reduced regressor
                            params_r: inertial parameters corresponding to W_e 
            Output: W_b: base regressor
                            phi_b: values of base parameters
                            numrank_W: numerical rank of regressor, determined by using a therehold
                            params_rsorted: inertial parameters included in base parameters """

    # scipy has QR pivoting using Householder reflection
    Q, R, P = linalg.qr(W_e, pivoting=True)

    # sort params as decreasing order of diagonal of R
    params_rsorted = []
    for i in range(P.shape[0]):
        print(i, ": ", params_r[P[i]], "---", abs(np.diag(R)[i]))
        params_rsorted.append(params_r[P[i]])

    # find rank of regressor
    numrank_W = 0
    # epsilon = np.finfo(float).eps  # machine epsilon
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

    # regrouping, calculating base params, base regressor
    print("rank of base regressor: ", numrank_W)
    R1 = R[0:numrank_W, 0:numrank_W]
    Q1 = Q[:, 0:numrank_W]
    R2 = R[0:numrank_W, numrank_W:R.shape[1]]

    # regrouping coefficient
    beta = np.around(np.dot(np.linalg.inv(R1), R2),
                     6)

    # values of base params
    phi_b = np.round(np.dot(np.linalg.inv(R1), np.dot(Q1.T, tau)), 6)
    # base regressor
    W_b = np.dot(Q1, R1)

    params_base = params_rsorted[:numrank_W]
    params_rgp = params_rsorted[numrank_W:]
    # print('regrouped params: ', params_rgp)
    tol_beta = 1e-6  # for scipy.signal.decimate
    for i in range(numrank_W):
        for j in range(beta.shape[1]):
            if abs(beta[i, j]) < tol_beta:
                params_base[i] = params_base[i]
            elif beta[i, j] < -tol_beta:
                params_base[i] = params_base[i] + ' - ' + \
                    str(abs(beta[i, j])) + '*' + str(params_rgp[j])
            else:
                params_base[i] = params_base[i] + ' + ' + \
                    str(abs(beta[i, j])) + '*' + str(params_rgp[j])
    base_parameters = dict(zip(params_base, phi_b))
    return W_b, base_parameters


def double_QR(tau, W_e, params_r):
    # scipy has QR pivoting using Householder reflection
    Q, R = np.linalg.qr(W_e)
    # print(pd.DataFrame(R[0:7,:]).to_latex())
    # sort params as decreasing order of diagonal of R
    assert np.diag(R).shape[0] == len(
        params_r), "params_r does not have same length with R"
    for i in range(np.diag(R).shape[0]):
        print(i, ": ", params_r[i], "---", abs(np.diag(R)[i]))
    idx_base = []
    idx_regroup = []
    # find rank of regressor
    epsilon = np.finfo(float).eps  # machine epsilon
    # tolpal = W_e.shape[0]*abs(np.diag(R).max())*epsilon#rank revealing tolerance
    tolpal = 0.02
    for i in range(len(params_r)):
        if abs(np.diag(R)[i]) > tolpal:
            # print(abs(np.diag(R)[i]))
            idx_base.append(i)
        else:
            # print(abs(np.diag(R)[i]))
            idx_regroup.append(i)

    numrank_W = len(idx_base)
    print("rank of base regressor: ", numrank_W)

    # rebuild W and params after sorted
    W1 = np.zeros([W_e.shape[0], len(idx_base)])
    W2 = np.zeros([W_e.shape[0], len(idx_regroup)])
    params_base = []
    params_regroup = []
    for i in range(len(idx_base)):
        W1[:, i] = W_e[:, idx_base[i]]
        params_base.append(params_r[idx_base[i]])
    for j in range(len(idx_regroup)):
        W2[:, j] = W_e[:, idx_regroup[j]]
        params_regroup.append(params_r[idx_regroup[j]])

    W_regrouped = np.c_[W1, W2]
    # perform QR on regrouped regressor
    Q_r, R_r = np.linalg.qr(W_regrouped)

    R1 = R_r[0:numrank_W, 0:numrank_W]
    Q1 = Q_r[:, 0:numrank_W]
    R2 = R_r[0:numrank_W, numrank_W:R.shape[1]]

    beta = np.around(np.dot(np.linalg.inv(R1), R2), 6)  # regrouping coefficient

    # values of base params
    phi_b = np.round(np.dot(np.linalg.inv(R1), np.dot(Q1.T, tau)), 6)
    W_b = np.dot(Q1, R1)  # base regressor

    print('regrouped params: ', params_regroup)
    tol_beta = 1e-4  # for scipy.signal.decimate
    for i in range(numrank_W):
        for j in range(beta.shape[1]):
            if abs(beta[i, j]) < tol_beta:
                params_base[i] = params_base[i]
            elif beta[i, j] < -tol_beta:
                params_base[i] = params_base[i] + ' - ' + \
                    str(abs(beta[i, j])) + '*' + str(params_regroup[j])
            else:
                params_base[i] = params_base[i] + ' + ' + \
                    str(abs(beta[i, j])) + '*' + str(params_regroup[j])

    # print('base parameters and their identified values: ')
    base_parameters = dict(zip(params_base, phi_b))
    # table = [params_base, phi_b]
    # print(tabulate(table))
    return W_b, base_parameters, params_base, phi_b


def relative_stdev(W_b, phi_b, tau):
    # stdev of residual error ro
    sig_ro_sqr = np.linalg.norm(
        (tau - np.dot(W_b, phi_b)))**2 / (W_b.shape[0] - phi_b.shape[0])
    # covariance matrix of estimated parameters
    C_x = sig_ro_sqr * np.linalg.inv(np.dot(W_b.T, W_b))
    # relative stdev of estimated parameters
    std_x_sqr = np.diag(C_x)
    std_xr = np.zeros(std_x_sqr.shape[0])
    for i in range(std_x_sqr.shape[0]):
        std_xr[i] = np.round(100 * np.sqrt(std_x_sqr[i]) / np.abs(phi_b[i]), 2)
    return std_xr


def visualization(robot):
    # If you want to visualize the robot in this example,
    # you can choose which visualizer to employ
    # by specifying an option from the command line:
    # GepettoVisualizer: -g
    # MeshcatVisualizer: -m

    VISUALIZER = None
    if len(argv) > 1:
        opt = argv[1]
        if opt == '-g':
            VISUALIZER = GepettoVisualizer
        elif opt == '-m':
            VISUALIZER = MeshcatVisualizer

    if VISUALIZER:
        robot.setVisualizer(VISUALIZER())
        robot.initViewer()
        robot.loadViewerModel("pinocchio")
        robot.display(robot.q0)


def identify_rand():
    # identification on random data points
    print("identification on random data")
    N_ = 1000
    q_rand , qd_rand, qdd_rand = generateWaypoints(N_, nq, nv, -1, 1)
    tau_rand= get_torque_rand(model, data, N_,  nq, nv, njoints, q_rand, qd_rand, qdd_rand, Ia, off, Iam6, fvm6, fsm6)
    # W_rand  = build_regressor_full(model, data, N_,  nq, nv,njoints, q_rand, qd_rand, qdd_rand)
    W_rand    = build_regressor_full_modified(model, data, N_,  nq, nv,njoints, q_rand, qd_rand, qdd_rand)
    print(W_rand.shape)
    W_rand    = add_coupling(W_rand,model, data, N_,  nq, nv,njoints, q_rand, qd_rand, qdd_rand)
    W_e, params_e, params_r = eliminateNonAffecting(W_rand, 1e-6)
    # W_b, base_parameters  = QR_pivoting(tau_rand, W_e, params_r)
    W_b, base_parameters  = double_QR(tau_rand, W_e, params_r)


 


if __name__ == '__main__':
    model = robot.model
    print(model)
    data = robot.data
    nq, nv, njoints = model.nq, model.nv, model.njoints
    # params_std = standardParameters(njoints,fv,fs,Ia,off,Iam6,fvm6,fsm6)
    params_std = standardParameters_modified(
        njoints, fv, fs, Ia, off, Iam6, fvm6, fsm6)
    # table_stdparams = pd.DataFrame(params_std.items(), columns=[
    #                                "Standard Parameters", "Value"])
    # print(table_stdparams)

    # print("identification on exp data")
    
    f_sample = 5000  
    curr_data = pd.read_csv('src/thanh/curr_data.csv').to_numpy()
    pos_data = pd.read_csv('src/thanh/pos_read_data.csv').to_numpy()
    # Nyquist freq/0.5*sampling rate fs = 0.5 *5 kHz

    N = pos_data.shape[0]
    y = np.zeros([N, curr_data.shape[1]])
    q = np.zeros([N, pos_data.shape[1]])

    y = curr_data
    q = pos_data

    # calculate joint position = inv(reduction ration matrix)*motor_encoder_angle
    red_q = np.diag([N1, N2, N3, N4, N5, N6])
    red_q[5, 4] = N6
    q_T = np.dot(np.linalg.inv(red_q), q.T)
    q = q_T.T

    # median order 3 => butterworth zerophase filtering
    nbutter = 4
    f_butter = 100
    b, a = signal.butter(nbutter, f_butter / (f_sample/2), 'low')
    for j in range(q.shape[1]):
        q[:, j] = signal.medfilt(q[:, j], 3)
        q[:, j] = signal.filtfilt(b, a, q[:, j])

    # calibration between joint mdh position and robot measure
    q[:, 1] += -np.pi / 2
    q[:, 2] += np.pi / 2
    # q[:, 5] += np.pi #already calibrated in urdf for joint 6

    # calculate vel and acc by central difference
    dt = 1 / f_sample
    dq = np.zeros([q.shape[0], q.shape[1]])
    ddq = np.zeros([q.shape[0], q.shape[1]])
    for i in range(pos_data.shape[1]):
        dq[:, i] = np.gradient(q[:, i], edge_order=2) / dt
        ddq[:, i] = np.gradient(dq[:, i], edge_order=2) / dt

    # suppress end segments of samples due to the border effect
    nbord = 5 * nbutter
    q = np.delete(q, np.s_[0:nbord], axis=0)
    q = np.delete(q, np.s_[(q.shape[0] - nbord):q.shape[0]], axis=0)
    dq = np.delete(dq, np.s_[0:nbord], axis=0)
    dq = np.delete(dq, np.s_[(dq.shape[0] - nbord):dq.shape[0]], axis=0)
    ddq = np.delete(ddq, np.s_[0:nbord], axis=0)
    ddq = np.delete(ddq, np.s_[(ddq.shape[0] - nbord):ddq.shape[0]], axis=0)

    # build regressor matrix
    qd = dq
    qdd = ddq
    N = q.shape[0]
    W = build_regressor_full_modified(
        model, data, N, nq, nv, njoints, q, qd, qdd)
    W = add_coupling(W, model, data, N, nq, nv, njoints, q, qd, qdd)

    # calculate joint torques = reduction gear ratio matrix*motor_torques
    red_tau = np.diag([N1, N2, N3, N4, N5, N6])
    red_tau[4, 5] = N6
    tau_T = np.dot(red_tau, y.T)

    # suppress end segments of  samples due to the border effect
    tau_T = np.delete(tau_T, np.s_[0:nbord], axis=1)
    tau_T = np.delete(tau_T, np.s_[(tau_T.shape[1] - nbord):tau_T.shape[1]], axis=1)

    # straight a matrix n-by-6 to a vector 6n-by-1
    tau = np.asarray(tau_T).ravel()

    # decimate by scipy.signal.decimate best_factor q = 25
    # parallel decimate joint-by-joint on joint torques and columns of regressor
    nj_ = tau.shape[0] // 6
    print("ni: ", nj_)
    tau_list = []
    W_list = []
    for i in range(nv):
        tau_temp = tau[(i * nj_):((i + 1) * nj_)]
        for m in range(2):
            print(tau_temp.shape)
            tau_temp = signal.decimate(tau_temp, q=10, zero_phase=True)
            print(tau_temp.shape)
        tau_list.append(tau_temp)
        W_joint_temp = np.zeros((tau_temp.shape[0], W.shape[1]))
        for j in range(W_joint_temp.shape[1]):
            W_joint_temp_col = W[(i * nj_):(i * nj_ + nj_), j]
            for n in range(2):
                W_joint_temp_col = signal.decimate(W_joint_temp_col, q=10, zero_phase=True)
            W_joint_temp[:, j] = W_joint_temp_col
        W_list.append(W_joint_temp)

    # eliminate qd crossing zero
    for i in range(len(W_list)):
        idx_qd_cross_zero = []
        for j in range(W_list[i].shape[0]):
            if abs(W_list[i][j, i * 14 + 11]) < qd_lim[i]:  # check columns of fv_i
                idx_qd_cross_zero.append(j)
        if i == 4 or i == 5:  # joint 5 and 6
            for k in range(W_list[i].shape[0]):
                if abs(W_list[i][k, 4 * 14 + 11] + W_list[i][k, 5 * 14 + 11]) < qd_lim[4] + qd_lim[5]:  # check sum cols of fv_5 + fv_6
                    idx_qd_cross_zero.append(k)
        # indices with vels around zero
        idx_eliminate = list(set(idx_qd_cross_zero))
        W_list[i] = np.delete(W_list[i], idx_eliminate, axis=0)
        tau_list[i] = np.delete(tau_list[i], idx_eliminate, axis=0)
        print(W_list[i].shape, tau_list[i].shape)

    # rejoining
    # note:length of data on each joint different
    row_size = 0
    for i in range(len(tau_list)):
        row_size += tau_list[i].shape[0]
   
    tau_ = np.zeros(row_size)
    W_ = np.zeros((row_size, W_list[0].shape[1]))
    a = 0
    for i in range(len(tau_list)):
        tau_[a:(a + tau_list[i].shape[0])] = tau_list[i]
        W_[a:(a + tau_list[i].shape[0]), :] = W_list[i]
        a += tau_list[i].shape[0]
    print(tau_.shape, W_.shape)

    #base parameters
    # elimate and QR decomposition for ordinary LS
    W_e, params_e, params_r = eliminateNonAffecting(W_, 0.001)
    W_b, base_parameters, params_base, phi_b = double_QR(tau_, W_e, params_r)
    std_xr_ols = relative_stdev(W_b, phi_b, tau_)
    phi_b_ols = np.around(np.linalg.lstsq(W_b, tau_, rcond=None)[0], 6)
    
    # weighted LS
    a = 0
    sig_ro_joint = np.zeros(nv)
    diag_SIGMA = np.zeros(row_size)
    # variance to each joint estimates
    for i in range(len(tau_list)):
        sig_ro_joint[i] = np.linalg.norm(tau_list[i] - np.dot(W_b[a:(a + tau_list[i].shape[0]), :], phi_b))**2 / (tau_list[i].shape[0])
        diag_SIGMA[a:(a + tau_list[i].shape[0])] = np.full(tau_list[i].shape[0], sig_ro_joint[i])
        a += tau_list[i].shape[0]
    SIGMA = np.diag(diag_SIGMA)
    # Covariance matrix
    C_X = np.linalg.inv(np.matmul(np.matmul(W_b.T, np.linalg.inv(SIGMA)), W_b))  # (W^T*SIGMA^-1*W)^-1
    # WLS solution
    phi_b = np.matmul(np.matmul(np.matmul(C_X, W_b.T), np.linalg.inv(SIGMA)), tau_)  # (W^T*SIGMA^-1*W)^-1*W^T*SIGMA^-1*TAU
    phi_b = np.around(phi_b, 6)

    #residual 
    print("number of equations(after preproccesing): ", row_size)
    print("residual norm: ", np.linalg.norm(tau_ - np.dot(W_b, phi_b)))
    print("relative residual norm: ", np.linalg.norm(tau_ - np.dot(W_b, phi_b))/np.linalg.norm(tau_ ))

    # WLS standard deviation
    STD_X = np.diag(C_X)
    std_xr = np.zeros(STD_X.shape[0])
    for i in range(STD_X.shape[0]):
        std_xr[i] = np.round(100 * np.sqrt(STD_X[i]) / np.abs(phi_b[i]), 2)

    # print("eleminanted parameters: ", params_e)
    print('condition number of base regressor: ', np.linalg.cond(W_b))
    # print('condition number of observation matrix: ', np.linalg.cond(W_e))

    
    path_save_bp = join(dirname(dirname(str(abspath(__file__)))),
                     "identification/src/thanh/TX40_bp_3.csv")
    with open(path_save_bp, "w") as output_file:
        w = csv.writer(output_file)
        for i in range(len(params_base)):
            w.writerow([params_base[i], phi_b_ols[i], std_xr_ols[i], phi_b[i], std_xr[i]])

    ############################################################################################################################
    #essential parameter
    min_std_e = min(std_xr)
    max_std_e = max(std_xr)
    std_xr_e = std_xr
    params_essential = params_base
    W_essential = W_b
    while not (max_std_e < ratio_essential*min_std_e):
        i, = np.where(np.isclose(std_xr_e, max_std_e))
        del params_essential[int(i)]
        W_essential = np.delete(W_essential, i, 1 )

        #OLS 
        phi_e_ols = np.around(np.linalg.lstsq(W_essential, tau_, rcond=None)[0], 6)
        std_e_ols = relative_stdev(W_essential, phi_e_ols, tau_)
        print('condition number of essential regressor: ', np.linalg.cond(W_essential))

        # weighted LS
        a = 0
        sig_ro_joint_e = np.zeros(nv)
        diag_SIGMA_e = np.zeros(row_size)
        # variance to each joint estimates
        for i in range(len(tau_list)):
            sig_ro_joint_e[i] = np.linalg.norm(tau_list[i] - np.dot(W_essential[a:(a + tau_list[i].shape[0]), :], phi_e_ols))**2 / (tau_list[i].shape[0])
            diag_SIGMA_e[a:(a + tau_list[i].shape[0])] = np.full(tau_list[i].shape[0], sig_ro_joint[i])
            a += tau_list[i].shape[0]
        SIGMA_e = np.diag(diag_SIGMA_e)
        # Covariance matrix
        C_X_e = np.linalg.inv(np.matmul(np.matmul(W_essential.T, np.linalg.inv(SIGMA_e)), W_essential))  # (W^T*SIGMA^-1*W)^-1
        # WLS solution
        phi_e_wls = np.matmul(np.matmul(np.matmul(C_X_e, W_essential.T), np.linalg.inv(SIGMA_e)), tau_)  # (W^T*SIGMA^-1*W)^-1*W^T*SIGMA^-1*TAU
        phi_e_wls = np.around(phi_e_wls, 6)
        # WLS standard deviation
        STD_X_e = np.diag(C_X_e)
        std_xr_e = np.zeros(STD_X_e.shape[0])
        for i in range(STD_X_e.shape[0]):
            std_xr_e[i] = np.round(100 * np.sqrt(STD_X_e[i]) / np.abs(phi_e_wls[i]), 2)
        min_std_e = min(std_xr_e)
        max_std_e = max(std_xr_e)
    print("number of equations(after preproccesing): ", row_size)
    print("residual norm: ", np.linalg.norm(tau_ - np.dot(W_essential, phi_e_wls)))
    print("relative residual norm: ", np.linalg.norm(tau_ - np.dot(W_essential, phi_e_wls))/np.linalg.norm(tau_ ))
    print('condition number of essential regressor: ', np.linalg.cond(W_essential))
    #save results to csv
    path_save_ep = join(dirname(dirname(str(abspath(__file__)))),
                     "identification/src/thanh/TX40_ep_3.csv")
    with open(path_save_ep, "w") as output_file:
        w = csv.writer(output_file)
        for i in range(len(params_essential)):
            w.writerow([params_essential[i], phi_e_ols[i],std_e_ols[i],phi_e_wls[i],std_xr_e[i]])
