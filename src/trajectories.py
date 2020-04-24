#!/usr/bin/env python3

import rospy
from copy import copy
import numpy as np
from STLtranslator import ReachAvoid
from nav_msgs.msg import OccupancyGrid
import matplotlib.pyplot as plt


# creating a occupancy grid with which to test the route optimization
def create_test_occupancy_grid():
	my_test_occupancy_grid = OccupancyGrid()

	# test map meta data
	my_test_occupancy_grid.info.width = 3
	my_test_occupancy_grid.info.height = 3
	my_test_occupancy_grid.info.resolution = 1.0

	# test map obstacles
	my_test_occupancy_grid.data = [0,100,0,0,100,0,0,0,0]

	plot_all_the_trajectories(my_test_occupancy_grid)

# the goal of the following three functions is to spit out a cost function that can be 
# optimized over using an established technique
def STL_signal(u):

	x0 = initial_state = np.asarray([-0.5,0,-0.5,0])[:,np.newaxis]

	# System definition: x_{t+1} = A*x_t + B*u_t
	A = np.array([[1,1,0,0],[0,1,0,0],[0,0,1,1],[0,0,0,1]])
	B = np.array([[0,0],[1,0],[0,0],[0,1]])

	T = u.shape[1]      # number of timesteps

	# Pre-alocate the signal
	s = np.zeros((4,T))

	# Run the controls through the system and see what we get
	x = copy(x0)
	for t in range(T):
		# extract the first and third elements of x
		s[0:2,t] = x[[0,2],:].flatten()
		s[2:4,t] = u[:,t]

		# Update the system state
		x = A@x + B@u[:,t][:,np.newaxis]   # ensure u is of shape (2,1) before applying

	return s

def plot_a_trajectory(u, ax):
	# displays the trajectory that has been found to be optimal
	state = STL_signal(u)

	x_coordinates = state[0,:]
	y_coordinates = state[1,:]

	ax.plot(x_coordinates, y_coordinates, linestyle="-", marker="o")

# generating an STL specification for the occupancy grid and computing and optimal trajectory through the region
def plot_all_the_trajectories(map):

	time_bound = 20
	goal = (1.5, 4)
	accuracy = 0.75
	time_steps = time_bound + 1

	my_reachavoid = ReachAvoid(map, time_bound, goal, accuracy)
	ax = my_reachavoid.return_region()

	# Opening up the file in which all the control iterations reside
	controls = np.loadtxt("controls.out", dtype=np.float32, delimiter=",")
	
	controls_list = []
	#modifying array such that each index is a control
	for index in range(np.shape(controls)[0]):
		if index % 2 == 0:
			controls_list.append( np.vstack( (controls[index], controls[index + 1]) ) )
		else:
			continue
		
	for index in range(len(controls_list)):
		if index % 50 == 0:
			print(controls_list[index])
			plot_a_trajectory(controls_list[index], ax)
		else:
			continue

	print(len(controls_list))
				
	plt.show()

if __name__ == '__main__':
	create_test_occupancy_grid()                                  
