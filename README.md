# OccupancyGridToSTL
##dependencies <br /> 
numpy <br />
pySTL <br />
matplotlib <br />
scipy <br />

##ROS dependencies <br /> 
map_server 

This is a ROS node which retrieves an occupany grid, generates an STL (i.e. signal temporal logic) specification 
from the retrieved grid, and then uses the specification to determine an optimal trajectory through the given region. 
 
The parameters for the optimization process can be accessed and changed in ROSstuff.py within the do_stuff_with_map method. 

If you want to write a new STL specification, create a new class in STLtranslator.py which inherits from the BaseSTLSpecification
class. 
 
##Changing Map Cell Dimensions <br /> 
[REMOTE] --> roscd turtlebot3_slam/launch/ <br />
[REMOTE] --> vim turtlebot3.gmapping.launch <br />
Once there scroll down to param name="delta" value="[desire_delta]" and input desired value  

##Creating Map <br /> 
[REMOTE] --> roscore <br />
[REMOTE] --> ssh pi@[PI_URI] <br />
[TURTLEBOT] --> roslaunch turtlebot3_bringup turtlebot3_robot.launch <br />
[REMOTE] --> roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping <br />
[REMOTE] --> rosrun turtlebot3_teleop turtlebot3_teleop_key <br />
[REMOTE] --> rosrun map_server map_saver -f ~/[map_name] <br />

##Run instructions <br /> 
In Home Directory (or where the map.yaml and map.pgm files are located) <br />
[REMOTE] -->roscore <br />
[REMOTE] -->rosrun map_server map_server [map_yaml_file] <br /> 
[REMOTE] -->rosrun OccupancyGridToSTL ROSstuff.py <br />

**Generating the optimal trajectory and displaying it may take several minutes 
depending on the optimization parameters and STL specification**


