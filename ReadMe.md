# 1. Introduction
   &ensp;&ensp;This repository contains the 3D model of xArm and demo packages for ROS development and simulations.Developing and testing environment: Ubuntu 16.04 + ROS Kinetic Kame.  
   Maintained by: Jimy (jimy.zhang@ufactory.cc) and Jason (jason@ufactory.cc)  
   ***Instructions below is based on xArm7, other model user can replace 'xarm7' with 'xarm6' or 'xarm5' where applicable.***

# 2. Update Summary
   This package is still in early development, tests, bug fixes and new functions are to be updated regularly in the future. 
   * Add xArm 7 description files, meshes and sample controller demos for ROS simulation and visualization.
   * Add Moveit! planner support to control Gazebo virtual model and real xArm, but the two can not launch together.
   * Direct control of real xArm through Moveit GUI is still in beta version, please use it with special care.
   * Add xArm hardware interface to use ROS position_controllers/JointTrajectoryController on real robot.
   * Add initial xArm 6 simulation support.

# 3. Preparations before using this package

## 3.1 Install the gazebo_ros interface module
   gazebo_ros_pkgs: <http://gazebosim.org/tutorials?tut=ros_installing>  
   ros_control: <http://wiki.ros.org/ros_control> (remember to select your correct ROS distribution)  
   
## 3.2 Go through the official tutorial documents
ROS Wiki: <http://wiki.ros.org/>  
Gazebo Tutorial: <http://gazebosim.org/tutorials>  
Gazebo ROS Control: <http://gazebosim.org/tutorials/?tut=ros_control>  

## 3.3 Download the 'table' 3D model
&ensp;&ensp;In Gazebo simulator, navigate through the model database for 'table' item, drag and place the 3D model inside the virtual environment. It will then be downloaded locally, as 'table' is needed running the demo.

# 4. Usage of ROS package 'xarm_ros'
   
## 4.1 Create a catkin workspace. 
   &ensp;&ensp;If you already have a workspace, skip and move on to next part.
   Follow the instructions in [this page](http://wiki.ros.org/catkin/Tutorials/create_a_workspace). 
   Please note that this readme instruction assumes the user continues to use '~/catkin_ws' as directory of the workspace.

## 4.2 Obtain the package
   ```bash
   $ cd ~/catkin_ws/src
   $ git clone https://github.com/xArm-Developer/xarm_ros.git
   ```

## 4.3 Build the code
   ```bash
   $ cd ~/catkin_ws
   $ catkin_make
   ```
## 4.4 Source the setup script
```bash
$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```
Skip above operation if you already have that inside your ~/.bashrc. Then do:
```bash
$ source ~/.bashrc
```
## 4.5 First try out in RViz:
```bash
$ roslaunch xarm_description xarm7_rviz_display.launch
```

## 4.6 Run the demo in Gazebo simulator
   ```bash
   $ roslaunch xarm_gazebo xarm7_beside_table.launch [run_demo:=true]
   ```
&ensp;&ensp;Add the run_demo option if you wish to see a pre-programed loop motion in action. The command trajectory is written in xarm_controller\src\sample_motion.cpp. And the trajectory in this demo is controlled by pure position interface.

# 5. Package structure
   
## 5.1 xarm_description
   &ensp;&ensp;xArm7 description files, mesh files and gazebo plugin configurations, etc. It's not recommended to change the xarm description file since other packages depend on it. 

## 5.2 xarm_gazebo
   &ensp;&ensp;Gazebo world description files and simulation launch files. User can add or build their own models in the simulation world file.

## 5.3 xarm_controller
   &ensp;&ensp;Controller configurations, robot command executable source, scripts and launch files. User can deploy their program inside this package or create their own.

### 5.3.1 xarm_controller/config
   Controller parameters to load into server, there are three basic types of controllers:  
   1) joint_state_controller/JointStateController: controller that publshes joint status, for Rviz or feedback.  
   2) effort_controllers/JointPositionController: position controller that has joint effort (torque) interface.  
   3) effort_controllers/JointEffortController: pure joint effort controller.  
   4) position_controllers/JointPositionController: pure position controller with only joint position interface.  
   These are examples for simulation purpose, when controlling the real arm, only position interface is provided. User can add their self-defined controllers as well, refer to: http://wiki.ros.org/ros_control (controllers)

### 5.3.2 xarm_controller/exec
  &ensp;&ensp;User can put their control scripts (shell, python, etc) here and can execute with 'rosrun' after setting them as executables.

### 5.3.3 xarm_controller/src, xarm_controller/include
   &ensp;&ensp;User can apply ROS API to program in C++ or python to make a program to control the virtual robot to move or monitor its status. Source files can be put here, remember to edit CMakeLists.txt before compiling. References:  [REF1](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29),  [REF2](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29)  


### 5.3.4 xarm_controller/launch
&ensp;&ensp;For launch files that bring about all steps for a simulation in proper order. User can refer to the file provided, load and initiate controllers from yaml configuration file and then run user application program to command robot arm to move.

## 5.4 xarm7_moveit_config
&ensp;&ensp;(Note that there may be some potential incompatibility issue with the STL format of the model, we will fix this problem soon)
   Generated by moveit_setup_assistant, could use with Moveit Planner and Rviz visualization. If you have Moveit! installed, you can try the demo. 
   ```bash
   $ roslaunch xarm7_moveit_config demo.launch
   ```

#### To run Moveit! motion planner along with Gazebo simulator:  
   First run:  
   ```bash
   $ roslaunch xarm_gazebo xarm7_beside_table.launch
   ```
   Then in another terminal:
   ```bash
   $ roslaunch xarm7_moveit_config xarm7_moveit_gazebo.launch
   ```
   If you have a satisfied motion planned in Moveit!, hit the "Execute" button and the virtual arm in Gazebo will execute the trajectory.

#### To run Moveit! motion planner to control the real xArm:  
   First, you should have the xArm-Python-SDK properly installed on your system, which is needed to command the hardware. Then make sure the xArm and the controller box is powered on, then execute:  
   ```bash
   $ roslaunch xarm7_moveit_config realMove_exec.launch robot_ip:=<your controller box LAN IP address>
   ```
   Examine the terminal output and see if any error occured during the launch. If not, just play with the robot in Rviz and you can execute the sucessfully planned trajectory on real arm. But be sure it will not hit any surroundings before execution! 

#### To launch the xarm simple motion planner together with the real xArm:  
```bash
   $ roslaunch xarm_planner xarm_planner_realHW.launch robot_ip:=<your controller box LAN IP address> robot_dof:=<7/6/5>
```
Argument 'robot_dof' specifies the number of joints of your xArm (default is 7). This implemented simple planner interface is based on move_group and provide service for users to do planning & execution based on the requested target, user can find detailed instructions on how to use it inside ***xarm_planner package***.