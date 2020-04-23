#!/usr/bin/env python3

import rospy
import numpy as np
from STLtranslator import ReachAvoid
from nav_msgs.msg import OccupancyGrid
from Optimizer import Optimizer

# creating a occupancy grid with which to test the route optimization
def create_test_occupancy_grid():
	my_test_occupancy_grid = OccupancyGrid()
	
	# test map meta data
	my_test_occupancy_grid.info.width = 3
	my_test_occupancy_grid.info.height = 3
	my_test_occupancy_grid.info.resolution = 1.0
	 
	# test map obstacles
	my_test_occupancy_grid.data = [0,100,0,0,0,0,100,100,100]

	do_stuff_with_map(my_test_occupancy_grid)

# generating an STL specification for the occupancy grid and computing and optimal trajectory through the region
def do_stuff_with_map(map):

	# the following code is exactly the same as in do_stuff_with_map in ROSstuff.py	

        # setting the parameters for the STL specification generator
        time_bound = 20
        goal = (0.5, 3.5)
        accuracy = 0.2
        time_steps = time_bound + 1

        # setting the parameters for the optimizer
        initial_state = np.asarray([0,0,0,0])[:,np.newaxis]
        u_guess = np.zeros((2, time_steps)).flatten()

        # optimization method
        method = 'Nelder-Mead'

        my_reachavoid = ReachAvoid(map, time_bound, goal, accuracy)
        ax = my_reachavoid.return_region()
        my_finished_specification = my_reachavoid.full_spec

        my_optimizer = Optimizer(initial_state, my_finished_specification, time_bound, time_steps, u_guess, ax)
        optimal_trajectory = my_optimizer.optimize(method)
        print("robustness: %s" % (my_optimizer.rho(optimal_trajectory)))
        my_optimizer.plot_trajectory(optimal_trajectory)	
        
        ''' 
        my_optimizer.plot_trajectory(np.asarray([[0.01330118, 0.01722613, 0.06619321, 0.0378914, 0.08764102, -0.01913139, 0.04494609, -0.01650196, -0.04589104, -0.03240899, 0.06242584, -0.00333347, -0.04715201, -0.02785758, 0.02584856, -0.07566842, -0.06440088, 0.05272855, -0.08763807, 0.00148727, -0.04923343], [0.03780754, 0.00369348, 0.02694225, -0.00477653, -0.01092062, -0.02311087, -0.01213407, 0.02055181, -0.00379446, 0.06702059, 0.0472324, 0.03970993, 0.03640072, 0.01647697, 0.02178436, -0.00282679, -0.02927775, -0.01250581, -0.05840783, -0.05287277, -0.01370572]])) 
        '''
	
        print(my_reachavoid.full_spec)

if __name__ == '__main__':
        create_test_occupancy_grid()
