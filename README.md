# OccupancyGridToSTL

## Dependencies <br />

### Python <br /> 
numpy <br />
pySTL <br />
matplotlib <br />
scipy <br />

***This project targeted Python 3***

### ROS <br /> 
map_server 

***This project targeted ROS Kinetic***

## Project Description <br />
Presently, this is a ROS node which retrieves an occupany grid, generates an STL (i.e. signal temporal logic) specification 
from the retrieved grid, and then uses the specification to determine an optimal trajectory through the given region. 

In the future, the goal is to create an initial map of the region, generate an STL specification, determine a route, and feed the 
route to a Turtlebot. Once the robot has recieved the map, it will begin moving through the route periodically re-optimizing the route
based upon its new position within the region. 

The next step with this project is to begin writing a function that formats the optimal control such that it can be fed to the Turtlebot. 

The parameters for the optimization process can be accessed and changed in ROSstuff.py within the do_stuff_with_map method. 

If you want to write a new STL specification, create a new class in STLtranslator.py which inherits from the BaseSTLSpecification
class. 

The three base files in this project are ROSstuff.py, Optimizer.py, and STLtranslator. ROSstuff creates instances of the Optimizer
and STLtranslator.  

Test.py can be used to create a custom occupancy grid without the use of SLAM and map_server. At the present moment, Trajectories.py
can be used to display the trajectories generated after running Optimizer (OptimizerSingleIntegrator is not yet supported). Optimizer
uses a double integrator control model whereas OptimizerSingleIntegrator uses a single integrator. Switching between the two models is 
achieved by changing which Optimizer is being instantiated (uncomment which one is imported in Test.py or ROSstuff.py) 
 
For information concerning the reason for the development of this project please consult the attached PDF

## Prerun Instructions <br/>

### Changing Map Cell Dimensions <br /> 
[REMOTE] --> roscd turtlebot3_slam/launch/ <br />
[REMOTE] --> vim turtlebot3.gmapping.launch <br />
Once there scroll down to param name="delta" value="[desired_delta]" and input desired value  

### Creating Map <br /> 
[REMOTE] --> roscore <br />
[REMOTE] --> ssh pi@[PI_URI] <br />
[TURTLEBOT] --> roslaunch turtlebot3_bringup turtlebot3_robot.launch <br />
[REMOTE] --> roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping <br />
[REMOTE] --> rosrun turtlebot3_teleop turtlebot3_teleop_key <br />
[REMOTE] --> rosrun map_server map_saver -f ~/[map_name] <br />

## Run Instructions <br />

### ROSstuff.py <br /> 
In Home Directory (or where the map.yaml and map.pgm files are located) <br />
[REMOTE] -->roscore <br />
[REMOTE] -->rosrun map_server map_server [map_yaml_file] <br /> 
[REMOTE] -->rosrun OccupancyGridToSTL ROSstuff.py <br />

### Test.py <br />
In Home Directory
[REMOTE] --> roscd OccupancyGridToSTL/src <br />
[REMOTE] --> python3 Test.py <br />

### Trajectories.py <br />
In Home Directory <br />
[REMOTE] --> roscd OccupancyGridToSTL/src <br />
[REMOTE] --> rm -rf controls.out <br />
[REMOTE] run either ROSstuff.py or Test.py <br />
**if running Test.py, make sure that the optimizer being used is the double integrator model** <br />
[REMOTE] --> python3 Trajectories.py <br />

**Generating the optimal trajectory and displaying it may take several minutes 
depending on the optimization parameters and STL specification**

**To terminate program --> first cancel the matplotlib window --> keyboard interrupt (control c) takes significantly longer** 

