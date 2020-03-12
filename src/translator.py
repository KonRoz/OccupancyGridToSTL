#!/usr/bin/env python3

import numpy as np
import rospy
from pySTL import STLFormula
from matplotlib import pyplot as plt
from matplotlib import colors
from matplotlib.patches import Rectangle
from nav_msgs.msg import OccupancyGrid

my_occupany_grid = OccupancyGrid()

# returns an STL formula indicating the signal is outside of the rectangle
def in_rectangle_formula(xmin,xmax,ymin,ymax):
	above_xmin = STLFormula(lambda s, t : s[t,0] - xmin)
	below_xmax = STLFormula(lambda s, t : -s[t,0] + xmax)
	above_ymin = STLFormula(lambda s, t : s[t,1] - ymin)
	below_ymax = STLFormula(lambda s, t : -s[t,1] + ymax)

	in_x_range = above_xmin.conjunction(below_xmax)
	in_y_range = above_ymin.conjunction(below_ymax)
	
	in_rectangle = in_x_range.conjunction(in_y_range)

	return in_rectangle

# this will be where the STL translation will actually take place
def generate_full_specification(occupied_cells):
	
	# list containing the x,y coordinates of the occupied grid cells
	objects_to_avoid = []
	
	# generating a STL formula for every occupied cell
	for cell in occupied_cells:
		# the x and y coordinates that are passed to this function are the lower left hand corner
		xmax = cell[0] + 1
		ymax = cell[1] + 1
		obstacle = in_rectangle_formula(cell[0], xmax, cell[1], ymax)
		# STL formula is currently defined as inside the obstacle --> not outside --> thus negation is used
		# The obstacles should always be avoided --> make sure t interval includes interesting part of signal
		obstacle_avoidance = obstacle.negation().always(0,84)
	 	# adding the individual STL formulas to an array
		objects_to_avoid.append(obstacle_avoidance)
	
	# this for loop is used to AND every individual formula within objects_to_avoid and create one comprehensive STL formula
	full_specification = objects_to_avoid[0]
	for obstacle in objects_to_avoid:
		full_specification = full_specification.conjunction(obstacle)
		
	return full_specification
 
# plotting a visual representation of grid and robustness functions
def do_stuff_with_map(map):
	# plotting the map
	fig, ax = plt.subplots(1)
	ax.set_xlim(0, map.info.width)
	ax.set_ylim(0, map.info.height)	
	
	occupied_cells = []
	
	# adding obstacles to the map 
	for cell in range(len(map.data)):
		# calculating x,y coordinates of lower left hand corner of cell
		y_coordinate = (int)(cell / map.info.width)
		x_coordinate = cell - (y_coordinate * map.info.width)
		
		# if a cell is occupied, the value is 100
		if map.data[cell] == 100:
			# plot the occupied cells on the map as red squares
			ax.add_patch ( Rectangle( (x_coordinate,y_coordinate),1,1,color='red', alpha=0.5) )
			# an array containing the lower left hand x & y coordinates of the occupied grid cells
			occupied_cells.append( (x_coordinate, y_coordinate) )
	
	# generating the robustness function for the cells contained within the occupied cells array  
	complete_function = generate_full_specification(occupied_cells)		
	
	"""

	Testing a few different routes to verify the usefulness of the generated robustness function
	
	--> a goal square is defined and a route is plotted

	"""
	
	ax.add_patch ( Rectangle( (30,30),1,1, color = 'green',alpha=0.5) ) 
	at_goal = in_rectangle_formula(30,31,30,31)
	goal_achieved = at_goal.eventually(0,84)
	full_spec = complete_function.conjunction(goal_achieved)
	
	t1 = np.array([[2.5*i,0.40*i] for i in range(85)])
	p1 = ax.scatter(t1[:,0],t1[:,1],label="$\\rho$ = %0.2f" % full_spec.robustness(t1,0))

	t2 = np.array([[2.5*i,0.03*i*i] for i in range(85)])
	p2 = ax.scatter(t2[:,0],t2[:,1],label="$\\rho$ = %0.2f" % full_spec.robustness(t2,0))
 
		
	plt.legend()
	plt.title("%sx%s w/ %s resolution" % (map.info.width, map.info.height, map.info.resolution))
	plt.show()

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
	

