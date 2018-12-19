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
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
// ROS
#include <ros/ros.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/JointState.h>

namespace xarm_control
{

class XArmHWInterface : public RobotHW
{
public:
	XArmHWInterface(unsigned int dof):dof_(dof){};
	~XArmHWInterface(){};
	bool init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh); 
	void read(const ros::Time& time, const ros::Duration& period);
	void write(const ros::Time& time, const ros::Duration& period);

	/* TODO:
	virtual bool prepareSwitch(const std::list<ControllerInfo>& start_list,
                             const std::list<ControllerInfo>& stop_list) { return true; }
  	virtual void doSwitch(const std::list<ControllerInfo>& ,
                        const std::list<ControllerInfo>& ) {}*/

private:
	unsigned int dof_;
	std::vector<std::string> jnt_names_;
	std::vector<double> position_cmd_;
	std::vector<double> velocity_cmd_;
	std::vector<double> effort_cmd_;

	std::vector<double> position_fdb_;
	std::vector<double> velocity_fdb_;
	std::vector<double> effort_fdb_;

	ros::Subscriber pos_sub_, vel_sub_, effort_sub_;
	void pos_fb_cb(sensor_msgs::JointState::::ConstPtr& data);

};

void XArmHWInterface::pos_fb_cb(sensor_msgs::JointState::::ConstPtr& data)
{
	for(int j=0; j<dof_; j++)
	{
		position_fdb_[j] = data->position[j];
		velocity_fdb_[j] = data->velocity[j];
		effort_fdb_[j] = data->effort[j];
	}
}

void XArmHWInterface::init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh)
{
	root_nh.subscribe("/joint_states", 100, &XArmHWInterface::pos_fb_cb, this);

}

void XArmHWInterface::read(const ros::Time& time, const ros::Duration& period)
{

}

void write(const ros::Time& time, const ros::Duration& period)
{
	// call servoj(joint_cmds[dof_]); // low level C++ api
}

} // namespace xarm_control


int main(int argc, char**argv)
{
	ros::init(argc, argv, "xarm_controller");
	ros::NodeHandle nh;

	xarm_control::XArmHWInterface xarm_hw(7);
	controller_manager::ControllerManager cm(&xarm_hw);

	

}

#endif