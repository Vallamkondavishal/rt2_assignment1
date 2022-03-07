# First Assignment of the Research Track 2 course - Main branch

## Behaviour of the software architecture

When the simulation start the user can type 1 to make the robot start. The robot continuosly reaches random generated poses till the user presses 0 then the robot firstly reaches the last generated random pose then it stops moving.

### launch folder

It contains:
1. [**sim.launch**]:a launch file to start the overall simulation of world and robot and all the nodes. 
2. [**simCop.launch**]: a launch file to start the four nodes, it is used to in the third point of the assignment to interact with Vrep.

## Projest Cop

The file [**projectCop.ttt**] contains the scene to upload on CoppeliaSim. A pioneer_ctrl robot is controlled through the four nodes of the given assignment.This can be possibally done by means of simulation which is fifth node that interacts with ROS nodes. In the childscripts a publisher to /odom topic has been declarated. This is used for publishing the actual position of the robot subscribed in node **go_to_point.py** . Moreover a subscriber to /cmd_vel is decleclared to retrieve the velocity of the robot set in the same node and actuate the motors.

## How to run the scene.
The package contains the nodes and the simulation environment for controlling a mobile robot in the CoppeliaSim simulation enviornment

To launch the node, please follow:
1. Open Coppeliasim in a sourced ROS terminal.
2. Load the scene on CoppeliaSim.
3. Launch on another terminal:

   ```
       roslaunch rt2_assignment1 simCop.launch
   ```