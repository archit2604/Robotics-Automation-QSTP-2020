# FINAL PROJECT
## Project Details:
- The task attempted is Task 1.1 i.e. Automation of Omnibase.
- The code stack uses PRM and A* Star search to plan the path, with the heuristic as euclidean distance.
- The controller used for the motion of the omnibase is a PID controller.
### The project submission has two versions project_fixed and project:
- project_fixed- The files with the name project_fixed are for the robotics navigation stack which makes the omnibase travel from (0,0) to (6,6).
- project- The files with the name projectare for the robotics navigation stack which makes the omnibase travel from anypoint to anypoint in the 6*6 grid in the first quadrant and starting at the origin. 

## Navigation Stack Contents:
### For project_fixed:
There are three ROS nodes in this stack(/project/src):
- obstacle.py-This is a obstacle detector node which publishes a fixed list of obstacles.
- planner_fixed.py-Path Planner node which subscribes to obstacles and publishes a path for (0,0) to (6,6).
- controller_fixed.py-Controller node which subscribes to the path and publishes cmd_vel.

This stack also has a roslaunch file. autonav_fixed.launch. This launch file runs all the above mentioned three nodes and also starts the omnibase in the given obstacle filled environment.It also launches a seperate terminal window to help track the progress of the stack.
### For project:
There are three ROS nodes in this stack(/project/src):
- obstacle.py-This is a obstacle detector node which publishes a fixed list of obstacles.
- planner.py-Path Planner node which subscribes to obstacles, asks the user for the goal in the 6*6 grid in the first quadrant and starting at the origin and publishes a path to the goal.After reaching the goal it asks the user whether he wants to make the bot travel to another point and continues to do so until the user says No.
- controller.py-Controller node which subscribes to the path and publishes cmd_vel.

This stack also has a roslaunch file. autonav.launch. This launch file runs all the above mentioned three nodes and also starts the omnibase in the given obstacle filled environment.It also launches a seperate terminal window to take inputs from the user.

## Rosbag files and Video Recordings:
The rosbag files for the simulations are in the folder /projectbag:
- project_fixed.bag file is a rosbag file which has the recorded values published to /cmd_vel for a simulation of the project_fixed stack from (0,0) to (6,6). 
- project.bag  file is a rosbag file which has the recorded values published to /cmd_vel for a simulation of the project stack for a path  [(0,0),(1,1),(2,2),(6,6),(0,0)]..

To view the videos for the simulations mentioned above use the links below:

1.[project_fixed.mp4](https://drive.google.com/file/d/1Om5-8K8zYb9nV32GJmOtDAoRQO5zS1Cn/view?usp=sharing)

2.[project.mp4](https://drive.google.com/file/d/194T3X2nGx4aL0TaW3E1d5iXrDL4LHSjj/view?usp=sharing)

