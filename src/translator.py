#!/usr/bin/env python3

import rospy
from pySTL import STLFormula
from matplotlib import pyplot as plt
from matplotlib import colors
from nav_msgs.msg import OccupancyGrid

my_occupany_grid = OccupancyGrid()

# the unprocessed occupancy grid is one large array --> this function takes in the raw occupancy grid and translates it into 
# an array with each row containing a number in a column
def grid_creator(big_array):
	# retrieving the dimensions of the map
	number_of_rows = big_array.info.height
	number_of_row_elements = big_array.info.width
	
	print(number_of_rows)
	print(number_of_row_elements)       
	 
	# slicing up the large array into rows and storing them as subarrays in an array called grid
	grid = []	
	n0 = 0
	for row in range(number_of_rows):
		n = ((row+1) * (number_of_row_elements))
		sliced = big_array.data[n0:n]
		grid.append(sliced)
		n0 = n

	print(grid)

	return grid

# this will be where the STL translation will actually take place
def do_stuff_with_map(map):
	grid = grid_creator(map)	
	
	# using matplotlib to display the area
	plt.imshow(grid, interpolation='none')
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
	

