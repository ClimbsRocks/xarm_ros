# Package Introduction
&ensp;&ensp;This package is intended to provide users a demo programming interface to use moveit!, instead of just using the GUI. To use the API better, users are encouraged to go through [Moveit tutorial](http://docs.ros.org/kinetic/api/moveit_tutorials/html/).  
&ensp;&ensp;Inside the package, 'xarm_simple_planner' is just a basic implementation of the Move_group interface, if higher level configurations (constraints, switch kinematic solver or planners, etc) are needed, user can fully explore Moveit abilities and implement a sofisticated version.

# Usage
First, to launch the simple planner node, run:  
```bash
   $ roslaunch xarm_planner xarm_planner_realHW.launch robot_ip:=[your controller box LAN IP address]
```
This node can provide services for planning request in Cartesian target and joint space target. Service definition can be found in srv folder. User can call the services to let planner solve the path to specified target point, and retrieve the boolean result as successful or not. Once the node is launched, user can try in command-line first, something like:  

For joint-space planning:  
```bash
   $ rosservice call joint_plan 'target: [1.0, -0.5, 0.0, -0.3, 0.0, 0.0, 0.5]'
```
Or, for Cartesian-space planning:  
```bash
   $ rosservice call pose_plan 'target: [position: [0.28, -0.2, 0.5], orientation: [0.0, 0.0, 0.0, 1.0]]'
```
After calling the two services, a boolean result named 'success' will be returned.  
If solution exists and user want to execute it on the robot, just publish a message (type: std_msgs/Bool) to the topic "/xarm_planner_exec", the boolean data should be 'true' to launch the execution:  
```bash
   $ rostopic pub -1 /xarm_planner_exec std_msgs/Bool 'true'
```
Alternative way of calling services or publish messages, is to do it programatically. User can refer to ROS [tutorial1](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29) and [tutorial2](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29) to find out how to do it, or refer to the 'xarm_simple_planner_test.cpp' in the src folder as an example.  
To run the test program, after launching the simple planner:
```bash
   $ rosrun xarm_planner xarm_simple_planner_test
```
The program will execute three hard-coded joint space target, ***MAKE SURE THERE ARE PLENTY SURROUNDING SPACES BEFORE EXECUTING THIS!***



