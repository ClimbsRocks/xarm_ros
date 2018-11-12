# 1. Introduction
   &ensp;&ensp;This repository contains the new xarm description file and 3D models for simulations based on ROS and Gazebo.
   Maintained by: Jimy (jimy92@163.com) and Jason (jason@ufactory.cc)

# 2. Preparations before using this package

## 2.1 install the gazebo_ros interface module
   gazebo_ros_pkgs: <http://gazebosim.org/tutorials?tut=ros_installing>  
   ros_control: <http://wiki.ros.org/ros_control> (remember to choose your ROS distribution)  
   
## 2.2 Go through the official tutorial documents
ROS Wiki: <http://wiki.ros.org/>  
Gazebo Tutorial: <http://gazebosim.org/tutorials>  
Gazebo ROS Control: <http://gazebosim.org/tutorials/?tut=ros_control>  

## 2.3 Download the 'table' 3D model
&ensp;&ensp;In Gazebo simulator, navigate through the model database for 'table' item, drag and place the 3D model inside the virtual environment. It will then be downloaded locally, as 'table' is needed running the demo.

# 3. Usage of ROS package 'xarm_robot'
   
## 3.1 Create a catkin workspace, if you have already done this, skip and move on to next part.
   &ensp;&ensp;Follow the instructions in [this page](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) 
   Please note that this readme instruction assumes the user continues to use '~/catkin_ws' as directory of the workspace.

## 3.2 Obtain the package
   ```bash
   $ cd ~/catkin_ws/src
   $ git clone https://github.com/uArm-Developer/ros_for_xarm.git
   ```

## 3.3 build the code
   ```bash
   $ cd ~/catkin_ws
   $ catkin_make
   ```

## 3.4 run the demo in Gazebo simulator
   ```bash
   $ roslaunch xarm_gazebo xarm7_beside_table.launch 
   ```
&ensp;&ensp;Motion will start after clicking on "play" button. The command trajectory is written in xarm_controller\src\sample_motion. And the arm in this demo is controlled by pure position interface.

# 4. Package structure
   
## 4.1 xarm_description
   &ensp;&ensp;xArm7 description files, mesh files and gazebo plugin configurations, etc. It's not recommended to change the xarm description file since other packages depend on it. 

## 4.2 xarm_gazebo
   &ensp;&ensp;Gazebo world description files and simulation launch files. User can add or build their own models in the simulation world file.

## 4.3 xarm_controller
   &ensp;&ensp;Controller configurations, robot command executable source, scripts and launch files. User can deploy their program inside this package or create their own.

### 4.3.1 xarm_controller/config
   Controller parameters to load into server, there are three basic types of controllers:  
   1) joint_state_controller/JointStateController: controller that publshes joint status, for Rviz or feedback.  
   2) effort_controllers/JointPositionController: position controller that has joint effort (torque) interface.  
   3) effort_controllers/JointEffortController: pure joint effort controller.  
   4) position_controllers/JointPositionController: pure position controller with only joint position interface.  
   User can add their self-defined controllers as well, refer to: http://wiki.ros.org/ros_control (controllers)

### 4.3.2 xarm_controller/exec
  &ensp;&ensp;User can put their control scripts (shell, python, etc) here and can execute with 'rosrun' after setting them as executables.

### 4.3.3 xarm_controller/src, xarm_controller/include
   &ensp;&ensp;User can apply ROS API to program in C++ or python to make a program to control the virtual robot to move or monitor its status. Source files can be put here, remember to edit CMakeLists.txt before compiling. Refer to:  
   <http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29>   
   <http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29>

### 4.3.4 xarm_controller/launch
&ensp;&ensp;For launch files that bring along all steps for a simulation in proper order. User can refer to the file provided, load and initiate controllers from yaml configuration file and then run user application program to command robot arm to move.

## 4.4 xarm7_moveit_config
   &ensp;&ensp;Generated by moveit_setup_assistant, could use with Moveit Planner and Rviz visualization. If you have Moveit! installed, you can try the demo.
   ```bash
   $ roslaunch xarm7_moveit_config demo.launch
   ```
