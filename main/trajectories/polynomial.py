import numpy as np 
import matplotlib.pylot as pylot
import math 
import time


class Polynomial():
	def __init__(self, njoints, Waypoints: list, Nf):
		self.njoints = njoints
		self.Nf =  Nf

		self.q0 = np.zeros(njoints)
		self.qd = np.zeros(njoints)
		self.qdd0 = np.zeros(njoints)

		self.ts = 0.01


		pass

	def Polynomial(self):
		pass

	def initConfig():
		self.q = []
	self.qd = []
		self.qdd = []


		pass

	def getCoeff_PTP():
		pass

	def plotTraj(self):
		pass