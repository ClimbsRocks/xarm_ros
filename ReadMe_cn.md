# 1. 简介：
   &ensp;&ensp;此代码库包含XArm模型文件以及相关的控制、规划等示例开发包。开发及测试使用的环境为 Ubuntu 16.04 + ROS Kinetic Kame。
   维护者: Jimy (jimy.zhang@ufactory.cc) Jason (jason@ufactory.cc)  
   ***以下的指令说明是基于xArm7, 其他型号用户可以在对应位置将'xarm7'替换成'xarm6'或'xarm5'***

# 2. 更新记录：
   此代码库仍然处在早期开发阶段，新的功能支持、示例代码，bug修复等等会保持更新。  
   * 添加Xarm 7 描述文档，3D图形文件以及controller示例，用于进行ROS可视化仿真模拟。
   * 添加MoveIt!规划器支持，用于控制Gazebo/RViz模型或者XArm真机，但二者不可同时启动。
   * 由ROS直接控制XArm真机的相关支持目前还是Beta版本，用户使用时应尽量小心，我们会尽快完善。
   * 添加 xArm hardware interface 并在驱动真实机械臂时使用 ROS position_controllers/JointTrajectoryController。
   * 添加 xArm 6 初版仿真支持。

# 3. 准备工作

## 3.1 安装 gazebo_ros interface 模块
   gazebo_ros_pkgs: <http://gazebosim.org/tutorials?tut=ros_installing>  
   ros_control: <http://wiki.ros.org/ros_control> (记得选择您使用的 ROS 版本)  
   
## 3.2 完整学习相关的官方教程
ROS Wiki: <http://wiki.ros.org/>  
Gazebo Tutorial: <http://gazebosim.org/tutorials>  
Gazebo ROS Control: <http://gazebosim.org/tutorials/?tut=ros_control>  

## 3.3 如果使用Gazebo: 请提前下载好 'table' 3D 模型
&ensp;&ensp;这个模型在Gazebo demo中会用到。在Gazebo仿真环境中, 在model database列表里寻找 'table', 并将此模型拖入旁边的3D环境中. 通过这个操作，桌子的模型就会自动下载到本地。

# 4. 'xarm_ros'的使用教程
   
## 4.1 生成catkin workspace. 
   &ensp;&ensp;如果您已经有了自己的catkin工作区，请跳过此步往下进行。
   按照[这里](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)的教程生成catkin_ws。 
   请留意本文档已假设用户继续沿用 '~/catkin_ws' 作为默认的catkin工作区地址。

## 4.2 获取代码包
   ```bash
   $ cd ~/catkin_ws/src
   $ git clone https://github.com/xArm-Developer/xarm_ros.git
   ```

## 4.3 编译代码
   ```bash
   $ cd ~/catkin_ws
   $ catkin_make
   ```
## 4.4 执行配置脚本
```bash
$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```
如果在您的 ~/.bashrc中已经有了以上语句，直接运行:
```bash
$ source ~/.bashrc
```
## 4.5 在RViz环境试用:
```bash
$ roslaunch xarm_description xarm7_rviz_display.launch
```
## 4.6 如果已安装Gazebo,可以执行demo查看效果
   ```bash
   $ roslaunch xarm_gazebo xarm7_beside_table.launch [run_demo:=true]
   ```
&ensp;&ensp;指定'run_demo'为true时Gazebo环境启动后机械臂会自动执行一套编好的循环动作。 这套简单的command trajectory写在xarm_controller\src\sample_motion.cpp. 这个demo加载的控制器使用position interface（纯位置控制）。

# 5. 代码包的结构
   
## 5.1 xarm_description
   &ensp;&ensp;包含xArm描述文档, mesh文件和gazebo plugin配置等等。 不推荐用户去修改urdf描述因为其他ros package对其都有依赖。

## 5.2 xarm_gazebo
   &ensp;&ensp;Gazebo world 描述文档以及仿真launch启动文档。用户可以在world中修改添加自己需要的模型与环境。

## 5.3 xarm_controller
   &ensp;&ensp;xarm使用的Controller配置, 轨迹指令源文件, 脚本以及launch文件。 用户可以基于这个包开发或者使用自己的package。以下是详细介绍：

### 5.3.1 xarm_controller/config
   需要导入ros server的控制器参数, 此demo提供三种可用的controller:  
   1) joint_state_controller/JointStateController: 发布关节状态的topic, 可用于Rviz显示或控制器获取反馈信息。  
   2) effort_controllers/JointPositionController: 通过关节力矩接口实现的位置控制器。  
   3) effort_controllers/JointEffortController: 纯关节力矩控制器（只有关节力矩命令/反馈接口）。  
   4) position_controllers/JointPositionController: 纯位置控制器（只有关节位置命令/反馈接口）。
   这些定义好的控制器仅用作仿真的例子, 当控制真实机械臂时只提供位置接口。用户可以根据需要添加自己的controller, 参考: http://wiki.ros.org/ros_control (controllers)

### 5.3.2 xarm_controller/exec
  &ensp;&ensp;用户可以将自己的控制程序 (shell, python, etc) 放在这里，将其编译或设置为executable后通过 'rosrun'命令执行。

### 5.3.3 xarm_controller/src, xarm_controller/include
   &ensp;&ensp;用户通过ROS API使用C++或python实现下发运动指令以及监控状态信息。相关源文件可以放在这里, 记得编译之前配置好CMakeLists.txt。 参考: [link1](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29), [link2](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29)  

### 5.3.4 xarm_controller/launch
&ensp;&ensp;放置启动文件：用于启动ros core并将需要的各个节点模块有序执行的文档。用户可以参考提供的launch文件，加载以及初始化控制器，或者运行带参数的ROS控制节点程序等等。

## 5.4 xarm7_moveit_config
&ensp;&ensp;
   部分文档由moveit_setup_assistant自动生成, 用于Moveit Planner和Rviz可视化仿真。如果已安装MoveIt!,可以尝试跑demo： 
   ```bash
   $ roslaunch xarm7_moveit_config demo.launch
   ```

#### Moveit!图形控制界面 + Gazebo 仿真环境:  
   首先执行:  
   ```bash
   $ roslaunch xarm_gazebo xarm7_beside_table.launch
   ```
   然后在另一个终端运行:
   ```bash
   $ roslaunch xarm7_moveit_config xarm7_moveit_gazebo.launch
   ```
   如果您在Moveit界面中规划了一条满意的轨迹, 点按"Execute"会使Gazebo中的虚拟机械臂同步执行此轨迹。

#### Moveit!图形控制界面 + xArm 真实机械臂:
   首先, 用户需要将 xArm-Python-SDK 正确安装到系统中才能正确控制真机。检查xArm电源和控制器已上电开启, 然后运行:  
   ```bash
   $ roslaunch xarm7_moveit_config realMove_exec.launch robot_ip:=<your controller box LAN IP address>
   ```
   检查terminal中的输出看看有无错误信息。如果启动无误，您可以将RViz中通过Moveit规划好的轨迹通过'Execute'按钮下发给机械臂执行。***但一定确保它不会与周围环境发生碰撞！***

#### 启动 xarm simple motion planner 控制 xArm 真实机械臂:  
```bash
   $ roslaunch xarm_planner xarm_planner_realHW.launch robot_ip:=<your controller box LAN IP address> robot_dof:=<7/6/5>
```
'robot_dof'参数指的是xArm的关节数目 (默认值为7)，这个简单实现的规划器接口是基于 move_group interface, 可以使用户通过service指定目标位置进行规划和执行。 这部分的详细使用方法请阅读***xarm_planner包***的***ReadMe***文档。