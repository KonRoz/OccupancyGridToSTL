#!/usr/bin/env python3

from copy import copy
import numpy as np
from scipy.optimize import minimize
from pySTL import STLFormula
import matplotlib.pyplot as plt

class Optimizer:
		
	def __init__(self, initial_state, full_spec, time_bound, time_steps, u_guess, ax):
		self.x0 = np.asarray(initial_state)
		self.full_spec = full_spec
		self.time_bound = time_bound
		self.time_steps = time_steps
		self.u_guess = np.asarray(u_guess)
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
	def STL_signal(self, u):
		""" 
		Maps a control signal u and an initial condition to an STL signal we can check. 
		This signal we will check is composed of the (x,y) position of the robot 
		and the control inputs.

		Arguments:
		u   : a (2,T) numpy array representing the control sequence

		Returns:
		s   : a (4,T) numpy array representing the signal we'll check
		"""
		# System definition: x_{t+1} = A*x_t + B*u_t
		A = np.array([[1,1,0,0],[0,1,0,0],[0,0,1,1],[0,0,0,1]])
		B = np.array([[0,0],[1,0],[0,0],[0,1]])

		T = u.shape[1]      # number of timesteps

		# Pre-alocate the signal
		s = np.zeros((4,T))

		# Run the controls through the system and see what we get
		x = copy(self.x0)
		for t in range(T):
			# extract the first and third elements of x
			s[0:2,t] = x[[0,2],:].flatten()
			s[2:4,t] = u[:,t]

			# Update the system state
			x = A@x + B@u[:,t][:,np.newaxis]   # ensure u is of shape (2,1) before applying

		return s

	def rho(self, u, spec=None):
		"""
		For a given initial state and control sequence u, calculates rho,
		a scalar value which indicates the degree of satisfaction of the specification.

		Arguments:
		u    : a (2,T) numpy array representing the control sequence
		spec : an STLFormula to evaluate (the full specification by default)

		Returns:
		rho  : a scalar value indicating the degree of satisfaction. Positive values
		indicate that the specification is satisfied.
		"""
		# By default, evaluate the full specification. Otherwise you could pass it 
		# a different formula, such as the (sub)specification for obstacle avoidance.
		if spec is None:
			spec = self.full_spec
		
		s = self.STL_signal(u)
		rho = spec.robustness(s.T, 0)

		return rho

	def cost_function(self, u):
		"""
		Defines a cost function over the control sequence u such that
		the optimal u maximizes the robustness degree of the specification.

		Arguments:
		u    : a (m*T,) flattened numpy array representing a tape of control inputs

		Returns:
		J    : a scalar value indicating the degree of satisfaction of the specification.
		(negative ==> satisfied)
		"""
		# enforce that the input is a numpy array
		u = np.asarray(u)

		# Reshape the control input to (mxT). Vector input is required for some optimization libraries
		T = int(len(u)/2)
		u = u.reshape((2,T))
		
		J = - self.rho(u)
		
		np.savetxt(self.control_output, u, delimiter=",")	
		print(J)
		print(u)
		return J

	
	def optimize(self, method):
		self.control_output = open("controls.out", "ab") 
 
		optimized = minimize(self.cost_function, self.u_guess,
				method=method,
				options={	
						'disp':True,
						'adaptive':True,
						'maxiter':30000,
						'fatol':1e-6,
						'xatol':1e-6
					}
			)
					
		u_optimal = optimized.x.reshape((2,self.time_steps))
		
		self.control_output.close()

		return u_optimal	
			
	def plot_trajectory(self, u):
		# displays the trajectory that has been found to be optimal
		state = self.STL_signal(u)

		x_coordinates = state[0,:]
		y_coordinates = state[1,:]

		self.ax.plot(x_coordinates, y_coordinates, linestyle="-", marker="o")

		plt.show()
