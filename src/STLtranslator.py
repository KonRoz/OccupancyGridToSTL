#!/usr/bin/env python3

from pySTL import STLFormula
from matplotlib import colors, pyplot as plt
from matplotlib.patches import Rectangle

# this class will serve as the base specification --> the one generated from the just the map
# more complicated specifications will inherit from this class and add to it
class BaseSTLSpecification:
	
	def __init__(self, map, time_bound):
		self.time_bound = time_bound
		self.map = map
		
		self.occupied_cells = []

                # adding obstacles to the map 
		for cell in range(len(self.map.data)):
                        # calculating x,y coordinates of lower left hand corner of cell
                        y_coordinate = (int)(cell / self.map.info.width)
                        x_coordinate = (int)(cell - (y_coordinate * self.map.info.width))

                        # if a cell is occupied, the value is 100
                        if self.map.data[cell] == 100:
                                # an array containing the lower left hand x & y coordinates of the occupied grid cells
                                self.occupied_cells.append( (x_coordinate * self.map.info.resolution, y_coordinate * self.map.info.resolution) ) 
		            
	# returns an STL formula indicating the signal is outside of the rectangle
	def in_rectangle_formula(self, xmin, xmax, ymin, ymax):
        	
		above_xmin = STLFormula(lambda s, t : s[t,0] - xmin)
		below_xmax = STLFormula(lambda s, t : -s[t,0] + xmax)
		above_ymin = STLFormula(lambda s, t : s[t,1] - ymin)
		below_ymax = STLFormula(lambda s, t : -s[t,1] + ymax)

		in_x_range = above_xmin.conjunction(below_xmax)
		in_y_range = above_ymin.conjunction(below_ymax)

		in_rectangle = in_x_range.conjunction(in_y_range)

		return in_rectangle

	# generates a base specification using for the obstacles, initial position, and goal position
	def generate_base_specification(self):

        	# list containing the x,y coordinates of the occupied grid cells		
		objects_to_avoid = []

        	# generating a STL formula for every occupied cell
		for cell in self.occupied_cells:
                	# the x and y coordinates that are passed to this function are the lower left hand corner
			xmax = cell[0] + self.map.info.resolution
			ymax = cell[1] + self.map.info.resolution
			obstacle = self.in_rectangle_formula(cell[0], xmax, cell[1], ymax)
			print('xmin = %s xmax = %s ymin = %s ymax = %s' % (cell[0], xmax, cell[1], ymax) ) 
                	# STL formula is currently defined as inside the obstacle --> not outside --> thus negation is used
                	# The obstacles should always be avoided --> make sure t interval includes interesting part of signal (i.e. the part of the trajectory
                	# that we are interested in analyzing the robustness score for
			obstacle_avoidance = obstacle.negation().always(0, self.time_bound)
                	# adding the individual STL formulas to an array
			objects_to_avoid.append(obstacle_avoidance)

        	# this for loop is used to AND every individual formula within objects_to_avoid and create one comprehensive STL formula
		full_specification = objects_to_avoid[0]
		for obstacle in objects_to_avoid:
			full_specification = full_specification.conjunction(obstacle)

		return full_specification

	# provides a graphical representation of the area's obstacles
	def add_obstacles_to_display(self):
                # plotting the map
		fig, ax = plt.subplots(1)
		ax.set_xlim(-1, self.map.info.width * self.map.info.resolution*2)
		ax.set_ylim(-1, self.map.info.height * self.map.info.resolution*2)

		for cell in self.occupied_cells:
			x_coordinate = cell[0]
			y_coordinate = cell[1]
			ax.add_patch(Rectangle( (x_coordinate,y_coordinate), self.map.info.resolution, self.map.info.resolution, color='red', alpha=0.5) )
		
		plt.title("%s by %s m Map " % ((int)(self.map.info.width * self.map.info.resolution), (int)(self.map.info.height * self.map.info.resolution)))
               
		return ax

class ReachAvoid(BaseSTLSpecification):

	def __init__(self, map, time_bound, goal, accuracy):
		super().__init__(map, time_bound)
		
		self.base_specification = super().generate_base_specification()		
		self.goal = goal
		self.accuracy = accuracy
		
		at_goal = super().in_rectangle_formula(self.goal[0] - self.accuracy, self.goal[0] + self.accuracy, self.goal[1] - self.accuracy, self.goal[1] + self.accuracy)
		goal_achieved = at_goal.eventually(0, time_bound)
               
		self.full_spec = self.base_specification.conjunction(goal_achieved)

	def return_region(self):
		ax = super().add_obstacles_to_display()
		
		# adding goal region to the displayed map
		ax.add_patch ( Rectangle( (self.goal[0] - self.accuracy, self.goal[1] - self.accuracy), 2*self.accuracy, 2*self.accuracy, color = 'green',alpha=0.5) )
	
		return ax


