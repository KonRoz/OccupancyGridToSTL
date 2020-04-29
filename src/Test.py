#!/usr/bin/env python3

import rospy
import numpy as np
from STLtranslator import ReachAvoid
from nav_msgs.msg import OccupancyGrid

# Comment whichever optimizer is not being used at the time --> also make sure to change x0 to a vector length of 4
#from OptimizerSingleIntegrator import OptimizerSingleIntegrator as Optimizer
from Optimizer import Optimizer

# creating a occupancy grid with which to test the route optimization
def create_test_occupancy_grid():
	my_test_occupancy_grid = OccupancyGrid()
	
	# test map meta data
	my_test_occupancy_grid.info.width = 3
	my_test_occupancy_grid.info.height = 3
	my_test_occupancy_grid.info.resolution = 1.0
	 
	# test map obstacles
	my_test_occupancy_grid.data = [0,100,0,0,100,0,0,0,0]

	do_stuff_with_map(my_test_occupancy_grid)

# generating an STL specification for the occupancy grid and computing and optimal trajectory through the region
def do_stuff_with_map(map):

	# the following code is exactly the same as in do_stuff_with_map in ROSstuff.py	

        # setting the parameters for the STL specification generator
        time_bound = 20
        goal = (3, 1)
        accuracy = 0.25
        time_steps = time_bound + 1

        # setting the parameters for the optimizer
        initial_state = np.asarray([0.5,0,0.5,0])[:,np.newaxis]
        u_guess = np.zeros((2, time_steps)).flatten()
        u_guess = np.asarray([[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],[0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1]])

        # optimization method
        method = 'Powell'

        my_reachavoid = ReachAvoid(map, time_bound, goal, accuracy)
        ax = my_reachavoid.return_region()
        my_finished_specification = my_reachavoid.full_spec

        my_optimizer = Optimizer(initial_state, my_finished_specification, time_bound, time_steps, u_guess, ax)
        optimal_trajectory = my_optimizer.optimize(method)
        print("robustness: %s" % (my_optimizer.rho(optimal_trajectory)))
        my_optimizer.plot_trajectory(optimal_trajectory)	
        
        print(my_reachavoid.full_spec)

if __name__ == '__main__':
        create_test_occupancy_grid()
