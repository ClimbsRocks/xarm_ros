#ifndef __XARM_ROS_CLIENT_H__
#define __XARM_ROS_CLIENT_H__

#include "ros/ros.h"
#include <xarm_driver.h>

namespace xarm_api{

class XArmROSClient
{
public:
	XArmROSClient(ros::NodeHandle& nh);
	~XArmROSClient(){};

	int motionEnable(short en);
	int setState(short state);
	int setMode(short mode);
	int setServoJ(const std::vector<float>& joint_cmd);

private:
	ros::ServiceClient motion_ctrl_client_;
	ros::ServiceClient set_mode_client_;
	ros::ServiceClient set_state_client_;
  	ros::ServiceClient go_home_client_;
	ros::ServiceClient move_lineb_client_;
	ros::ServiceClient move_servoj_client_;

    xarm_msgs::SetAxis set_axis_srv_;
    xarm_msgs::SetInt16 set_int16_srv_;
    xarm_msgs::Move move_srv_;

    ros::NodeHandle nh_;
};

}

#endif
