#include <xarm_ros_client.h>

namespace xarm_api{

XArmROSClient::XArmROSClient(ros::NodeHandle& nh)
{   
    nh_ = nh;
	motion_ctrl_client_ = nh_.serviceClient<xarm_msgs::SetAxis>("motion_ctrl");
	set_mode_client_ = nh_.serviceClient<xarm_msgs::SetInt16>("set_mode");
	set_state_client_ = nh_.serviceClient<xarm_msgs::SetInt16>("set_state");
  	go_home_client_ = nh_.serviceClient<xarm_msgs::Move>("go_home");
	move_lineb_client_ = nh_.serviceClient<xarm_msgs::Move>("move_lineb");
	move_servoj_client_ = nh_.serviceClient<xarm_msgs::Move>("move_servoj",true); // persistent connection for servoj

}

int XArmROSClient::motionEnable(short en)
{
	set_axis_srv_.request.id = 8;
    set_axis_srv_.request.data = en;
    if(motion_ctrl_client_.call(set_axis_srv_))
    {
        ROS_INFO("%s\n", set_axis_srv_.response.message.c_str());
        return set_axis_srv_.response.ret;
    }
    else
    {
        ROS_ERROR("Failed to call service motion_ctrl");
        return 1;
    }

}

int XArmROSClient::setState(short state)
{
	set_int16_srv_.request.data = state;
    if(set_state_client_.call(set_int16_srv_))
    {
        ROS_INFO("%s\n", set_int16_srv_.response.message.c_str());
        return set_int16_srv_.response.ret;
    }
    else
    {
        ROS_ERROR("Failed to call service set_state");
        return 1;
    }
}

int XArmROSClient::setMode(short mode)
{
	set_int16_srv_.request.data = mode;
    if(set_mode_client_.call(set_int16_srv_))
    {
        ROS_INFO("%s\n", set_int16_srv_.response.message.c_str());
        return set_int16_srv_.response.ret;
    }
    else
    {
        ROS_ERROR("Failed to call service set_mode");
        return 1;
    }  

}

int XArmROSClient::setServoJ(const std::vector<float>& joint_cmd)
{
	move_srv_.request.mvvelo = 0;
    move_srv_.request.mvacc = 0;
    move_srv_.request.mvtime = 0;
    move_srv_.request.pose = joint_cmd;

    if(move_servoj_client_.call(move_srv_))
    {
        // ROS_INFO("%s\n", move_srv_.response.message.c_str());
        return move_srv_.response.ret;
    }
    else
    {
        ROS_ERROR("Failed to call service move_servoj");
        return 1;
    }
}

}
