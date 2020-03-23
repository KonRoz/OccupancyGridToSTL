#!/usr/bin/env python3

from copy import copy
import numpy as np
from scipy.optimize import minimize
from pySTL import STLFormula
import matplotlib.pyplot as plt

class Optimizer:
		
	def __init__(self, initial_state, full_spec, time_bound, ax):
		self.initial_state = np.asarray(initial_state)
		self.full_spec = full_spec
		self.time_bound = time_bound
		self.ax = ax
		
		# setting up the control constraints --> rectangle for 2 controls (u1 & u2)
		# lambda should output positive in desired scenario
		u_min = -0.2
		u_max = 0.2
		u1_above_min = STLFormula(lambda s, t : s[t,2] - u_min)
		u1_below_max = STLFormula(lambda s, t : -s[t,2] + u_max)
		u2_above_min = STLFormula(lambda s, t : s[t,3] - u_min)
		u2_below_max = STLFormula(lambda s, t : -s[t,3] + u_max)
        	
		# making sure the control stays positive inside the desired rectangle via AND
		u1_valid = u1_above_min.conjunction(u1_below_max)
		u2_valid = u2_above_min.conjunction(u2_below_max)

		self.bounded_control = u1_valid.conjunction(u2_valid).always(0,self.time_bound)

		# formulating the complete specification
		self.full_spec = self.full_spec.conjunction(self.bounded_control)
	
	# the goal of the following three functions is to spit out a cost function that can be 
	# optimized over using an established technique
	def determine_state(self, u):
		# given an initial state and control --> determine the state of the system
		A = np.array([[1,1,0,0],[0,1,0,0],[0,0,1,1],[0,0,0,1]])
		B = np.array([[0,0],[1,0],[0,0],[0,1]])

		time_steps = u.shape[1]

		signal = np.zeros((4,time_steps))

		x = copy(self.initial_state)
		for t in range(time_steps):
			signal[0:2,t] = x[[0,2],:].flatten()
			signal[2:4,t] = u[:,t]

			x = A@x + B@u[:,t][:,np.newaxis]

		return signal

	def rho(self, u):
		# spit out a robustness function for the cost function
		signal = self.determine_state(u)
		rho = self.full_spec.robustness(signal.T, 0)
		
		return rho

	def cost_function(self, u):
		# packages up the robustness function so that it can be fed to the optimizer
		
		# u is the flattened control sequence --> dim = 1xmT
		u = np.asarray(u)
		
		# determining the number of time steps in the control sequence and using it to reshape u 
		time_steps = int(len(u)/2)
		u = u.reshape((2,time_steps))
		
 		# rho computes the robustness value for the particular number of time steps
		cost_function = - self.rho(u)

		# the cost function is what can be optimized using scipy
		return cost_function
	
	def optimize(self, method):
		# takes a guess and a method and spits out an optimal trajectory
		u_guess = np.zeros((2,41)).flatten() 

		optimized = minimize(self.cost_function, u_guess,
				method=method,
				options={	
						'disp':True,
						'adaptive':True,
						'maxiter':10000,
						'fatol':1e-6,
						'xatol':1e-6
					}
			)
					
		u_optimal = optimized.x.reshape((2,41))

		return u_optimal	
			
	def plot_trajectory(self, u):
		# displays the trajectory that has been found to be optimal
		state = self.determine_state(u)

		x_coordinates = state[0,:]
		y_coordinates = state[1,:]

		self.ax.scatter(x_coordinates, y_coordinates)

		plt.show()
