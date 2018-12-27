/* Copyright 2018 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jason Peng <jason@ufactory.cc>
 ============================================================================*/
#ifndef __XARM_HARDWARE_INTERFACE_H__
#define __XARM_HARDWARE_INTERFACE_H__

// ros_control
#include <control_toolbox/pid.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
// ROS
#include <ros/ros.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/JointState.h>
// for mutex
#include <pthread.h>
#include "xarm/connect.h"
#include "xarm/instruction/uxbus_cmd_config.h"

namespace xarm_control
{

const std::string jnt_state_topic = "joint_states";

class XArmHWInterface : public hardware_interface::RobotHW
{
public:
	XArmHWInterface(unsigned int dof, const std::string& robot_ip, ros::NodeHandle &root_nh);
	~XArmHWInterface();
	// bool init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh); 
	void read(/*const ros::Time& time, const ros::Duration& period*/);
	void write(/*const ros::Time& time, const ros::Duration& period*/);

	// ros::Time get_time();
	// ros::Duration get_period();

	/* TODO:
	virtual bool prepareSwitch(const std::list<ControllerInfo>& start_list,
                             const std::list<ControllerInfo>& stop_list) { return true; }
  	virtual void doSwitch(const std::list<ControllerInfo>& ,
                        const std::list<ControllerInfo>& ) {}*/

private:
	unsigned int dof_;
	std::vector<std::string> jnt_names_ = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};
	std::vector<double> position_cmd_;
	std::vector<double> velocity_cmd_;
	std::vector<double> effort_cmd_;

	std::vector<double> position_fdb_;
	std::vector<double> velocity_fdb_;
	std::vector<double> effort_fdb_;

	// ros::Time last_called_time, this_called_time;
	UxbusCmd *arm_cmd;

	// pthread_mutex_t mutex_cmd = PTHREAD_MUTEX_INITIALIZER, mutex_fdb = PTHREAD_MUTEX_INITIALIZER;
	// pthread_mutex_lock(&mutex);
 	// pthread_mutex_unlock(&mutex);

	hardware_interface::JointStateInterface    js_interface_;
  	hardware_interface::EffortJointInterface   ej_interface_;
  	hardware_interface::PositionJointInterface pj_interface_;
  	hardware_interface::VelocityJointInterface vj_interface_;

	ros::Subscriber pos_sub_, vel_sub_, effort_sub_;
	void pos_fb_cb(const sensor_msgs::JointState::ConstPtr& data);

};

XArmHWInterface::XArmHWInterface(unsigned int dof, const std::string& robot_ip, ros::NodeHandle &root_nh)
{
	dof_=dof;
	position_cmd_.resize(dof_);
	position_fdb_.resize(dof_);
	velocity_cmd_.resize(dof_);
	velocity_fdb_.resize(dof_);
	effort_cmd_.resize(dof_);
	effort_fdb_.resize(dof_);

	pos_sub_ = root_nh.subscribe(jnt_state_topic, 100, &XArmHWInterface::pos_fb_cb, this);

	// ros::Duration(2).sleep();

	for(unsigned int j=0; j < dof_; j++)
  	{
  		// Create joint state interface for all joints
    	js_interface_.registerHandle(hardware_interface::JointStateHandle(jnt_names_[j], &position_fdb_[j], &velocity_fdb_[j], &effort_fdb_[j]));

    	hardware_interface::JointHandle joint_handle;
    	joint_handle = hardware_interface::JointHandle(js_interface_.getHandle(jnt_names_[j]),&position_cmd_[j]);
      	pj_interface_.registerHandle(joint_handle);
  	}

  	registerInterface(&js_interface_);
  	registerInterface(&pj_interface_);


  	// xArm:
  	char ip_non_const[50];
  	strcpy(ip_non_const,robot_ip.c_str());

  	arm_cmd = connect_tcp_control(ip_non_const);

  	arm_cmd->motion_en(8, 1);

  	arm_cmd->set_mode(XARM_MODE::SERVO);

  	arm_cmd->set_state(XARM_STATE::START);


}

XArmHWInterface::~XArmHWInterface()
{
	arm_cmd->set_mode(XARM_MODE::POSE);
	// arm_cmd->set_state(XARM_STATE::START);
	arm_cmd->close();
	fprintf(stderr, "Finish, XArm Disconnected.\n");
}

void XArmHWInterface::pos_fb_cb(const sensor_msgs::JointState::ConstPtr& data)
{

	for(int j=0; j<dof_; j++)
	{
		position_fdb_[j] = data->position[j];
		velocity_fdb_[j] = data->velocity[j];
		effort_fdb_[j] = data->effort[j];
	}
}

// void XArmHWInterface::init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh)
// {
// 	root_nh.subscribe("/joint_states", 100, &XArmHWInterface::pos_fb_cb, this);

// }

void XArmHWInterface::read(/*const ros::Time& time, const ros::Duration& period*/)
{

}

void XArmHWInterface::write(/*const ros::Time& time, const ros::Duration& period*/)
{
	// call servoj(joint_cmds[dof_]); // low level C++ api or call service
	float mvjoint[dof_];
	for(int k=0; k<dof_; k++)
	{
		mvjoint[k] = (float)position_cmd_[k];
	}

	arm_cmd->move_servoj(mvjoint, 0, 0, 0);

	// if(fabs(position_cmd_[0]>0.45))
	// {
	// 	fprintf(stderr, "set_servoj:\t");
	// 	for(int i=0; i<dof_; i++)
	// 	{
	// 		fprintf(stderr, "%lf\t", position_cmd_[i]);
	// 	}
	// 	fprintf(stderr, "\n");
	// }
}

// ros::Time XArmHWInterface::get_time()
// {
// 	this_called_time = ros::Time::now();
// 	return this_called_time;
// }

// ros::Duration XArmHWInterface::get_period()
// {
// 	ros::Duration tmp = this_called_time - last_called_time;
// 	last_called_time = this_called_time;
// 	return tmp;
// }

} // namespace xarm_control


int main(int argc, char**argv)
{
	ros::init(argc, argv, "xarm_controller");
	ros::NodeHandle nh;
	ros::Rate r(100); // ServoJ mode can not handle update rate greater than 100Hz
	std::string ip="192.168.1.121";
	xarm_control::XArmHWInterface xarm_hw(7, ip, nh);
	controller_manager::ControllerManager cm(&xarm_hw, nh);

  	ros::AsyncSpinner spinner(4);
	spinner.start();

	// IMPORTANT: DO NOT REMOVE THIS DELAY !!!
	/* Wait for correct initial position to be updated to ros_controller */
	ros::Duration(2.0).sleep();

	ros::Time ts = ros::Time::now();
	while (ros::ok())
	{	
	   ros::Duration elapsed = ros::Time::now() - ts;
	   ts = ros::Time::now();
	   // xarm_hw.read();
	   // cm.update(xarm_hw.get_time(), xarm_hw.get_period());
	   cm.update(ts, elapsed);
	   xarm_hw.write();
	   r.sleep();
	}
	spinner.stop();
	return 0;
}

#endif