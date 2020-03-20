#!/usr/bin/env python3

import numpy as np
from pySTL import STLFormula

class Optimizer:
		
	def __init__(self, initial_state, full_spec, time_bound):
		self.initial_state = np.asarray(initial_state)
		self.full_spec = full_spec
		self.time_bound = time_bound
		
		# setting up the control constraints --> rectangle for 2 controls (u1 & u2)
		# lambda should output positive in desired scenario
		u_min = -0.2
		u_max = 0.2
		u1_above_min = STLFormula(lambda s, t : s[t,2] - umin)
		u1_below_max = STLForumla(lambda s, t : -s[t,2] + umax)
		u2_above_min = STLForumla(lambda s, t : s[t,3] - umin)
		u2_below_max = STLForumla(lambda s, t : -s[t,3] + umax)
        	
		# making sure the control stays positive inside the desired rectangle via AND
		u1_valid = u1_above_min.conjunction(u1_below_max)
		u2_valid = u2_above_min.conjunction(u2_below_max)

		self.bounded_control = u1_valid.conjunction(u2_valid).always(0,self.time_bound)

		# formulating the complete specification
		self.full_spec = self.full_spec.conjunction(self.bounded_control)
	
	def optimize(self, method):
		pass				
		
		
