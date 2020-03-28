# OccupancyGridToSTL
dependencies: <br /> 
numpy 
pySTL 
matplotlib 
scipy 

ROS dependencies: <br /> 
map_server 

This is a ROS node which retrieves an occupany grid, generates an STL (i.e. signal temporal logic) specification 
from the retrieved grid, and then uses the specification to determine an optimal trajectory through the given region. 
 
The parameters for the optimization process can be accessed and changed in ROSstuff.py within the do_stuff_with_map method. 

If you want to write a new STL specification, create a new class in STLtranslator.py which inherits from the BaseSTLSpecification
class. 
 
Changing Map Cell Dimensions: <br /> 
[REMOTE] --> roscd turtlebot3_slam/launch/ 
[REMOTE] --> vim turtlebot3.gmapping.launch
Once there scroll down to the <param name="delta" value="[desire_delta]"/> and input delta  

Creating Map: <br /> 
[REMOTE] --> roscore
[REMOTE] --> ssh pi@[PI_URI]
[TURTLEBOT] --> roslaunch turtlebot3_bringup turtlebot3_robot.launch
[REMOTE] --> roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping 
[REMOTE] --> rosrun turtlebot3_teleop turtlebot3_teleop_key 
[REMOTE] --> rosrun map_server map_saver -f ~/[map_name]

Run instructions: <br /> 
In Home Directory (or where the map.yaml and map.pgm files are located)
[REMOTE] -->roscore 
[REMOTE] -->rosrun map_server map_server [map_yaml_file] 
[REMOTE] -->rosrun OccupancyGridToSTL ROSstuff.py 

**Generating the optimal trajectory and displaying it may take several minutes 
depending on the optimization parameters and STL specification**


