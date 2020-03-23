#!/usr/bin/env python3
import rospy
import numpy as np
from STLtranslator import ReachAvoid
from nav_msgs.msg import OccupancyGrid
from Optimizer import Optimizer


my_occupany_grid = OccupancyGrid()

# generating an STL specification for the occupancy grid and computing and optimal trajectory through the region
def do_stuff_with_map(map):
	# setting the parameters for the STL specification generator
	time_bound = 40
	goal = (20,10)
	accuracy = 1
	time_steps = time_bound + 1
		
	# setting the parameters for the optimizer
	initial_state = np.array([1,1,0,0])[:,np.newaxis] 
	u_guess = np.zeros((2, time_steps))
	
	# optimization method
	method = 'CG'

	my_reachavoid = ReachAvoid(map, time_bound, goal, accuracy)
	ax = my_reachavoid.return_region()
	my_finished_specification = my_reachavoid.full_spec

	my_optimizer = Optimizer(initial_state, my_finished_specification, time_bound, ax)
	optimal_trajectory = my_optimizer.optimize(method)
	print(optimal_trajectory)
	my_optimizer.plot_trajectory(optimal_trajectory)

# callback function for the map_subscriber --> stores the read occupancy grid in a global occupancy grid
def map_listener_callback(data):
	# if a function changes the value of a global variable --> must be declared at the beginning of the function
	global my_occupany_grid
	my_occupany_grid = data

def map_listener():
	
	# initializing node
	STL_node = rospy.init_node('map_listener', anonymous=True)
	
	# detailing which topic will be subscribed to & the message type that should be listened for
	map_subscriber = rospy.Subscriber('/map', OccupancyGrid, map_listener_callback) 	
 	
	# allow the map_subcriber to read the grid from the map topic and store it in my_occupancy_grid
	# without this statement, the subscriber does not have time to read the occupancy grid from the map
	# server and thus stores and empty grid in my_occupancy_grid	
	rospy.sleep(1)	
       	
	# ensure that the occupancy grid has been read
	if my_occupany_grid.info.resolution != 0.0:
		do_stuff_with_map(my_occupany_grid)
		print('success')
	else:
		rospy.signal_shutdown('something went wrong and the occupancy grid is empty')
			
	# keep node running without exiting
	rospy.spin()

# creating the map_listener node in the case that this is the file being run 
if __name__ == '__main__':
	map_listener()
	

