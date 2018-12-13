/* Copyright 2018 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jason Peng <jason@ufactory.cc>
 ============================================================================*/
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <std_msgs/Bool.h>
#include <xarm_planner/pose_plan.h>
#include <xarm_planner/joint_plan.h>

#define SPINNER_THREAD_NUM 2

static const std::string PLANNING_GROUP("xarm7");

class XArmSimplePlanner
{
  public:
    XArmSimplePlanner(const std::string plan_group_name):spinner(SPINNER_THREAD_NUM), group(plan_group_name){init();};
    XArmSimplePlanner():spinner(SPINNER_THREAD_NUM),group(PLANNING_GROUP){init();};
    ~XArmSimplePlanner(){};
    void start();
    void stop();

  private:
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    std::vector<std::string> joint_names;
    moveit::planning_interface::MoveGroupInterface group;

    ros::Publisher display_path;
    ros::ServiceServer plan_pose_srv;
    ros::ServiceServer plan_joint_srv;
    ros::Subscriber exec_plan_sub; /* non-blocking*/
    ros::ServiceServer exec_plan_srv; /* blocking with result feedback */

    void init();
    bool do_pose_plan(xarm_planner::pose_plan::Request &req, xarm_planner::pose_plan::Response &res);
    bool do_joint_plan(xarm_planner::joint_plan::Request &req, xarm_planner::joint_plan::Response &res);
    void execute_plan_topic(const std_msgs::Bool::ConstPtr& exec);
};

void XArmSimplePlanner::init()
{
  joint_names = group.getJointNames();

  display_path = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true); /*necessary?*/

  ROS_INFO_NAMED("move_group_planner", "Reference frame: %s", group.getPlanningFrame().c_str());

  ROS_INFO_NAMED("move_group_planner", "End effector link: %s", group.getEndEffectorLink().c_str());

  /* Notice: the correct way to specify member function as callbacks */
  plan_pose_srv = node_handle.advertiseService("pose_plan", &XArmSimplePlanner::do_pose_plan, this);
  plan_joint_srv = node_handle.advertiseService("joint_plan", &XArmSimplePlanner::do_joint_plan, this);

  exec_plan_sub = node_handle.subscribe("xarm_planner_exec", 10, &XArmSimplePlanner::execute_plan_topic, this);

}

void XArmSimplePlanner::start()
{
  ROS_INFO("Spinning");
  spinner.start();
}

void XArmSimplePlanner::stop()
{
  spinner.stop();
}

bool XArmSimplePlanner::do_pose_plan(xarm_planner::pose_plan::Request &req, xarm_planner::pose_plan::Response &res)
{
  group.setPoseTarget(req.target);
  
  ROS_INFO("xarm_planner received new target: [ position: (%f, %f, %f), orientation: (%f, %f, %f, %f)", \
    req.target.position.x, req.target.position.y, req.target.position.z, req.target.orientation.x, \
    req.target.orientation.y, req.target.orientation.z, req.target.orientation.w);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  res.success = success;
  ROS_INFO_NAMED("move_group_planner", "This plan (pose goal) %s", success ? "SUCCEEDED" : "FAILED");
  
  return success;
}

bool XArmSimplePlanner::do_joint_plan(xarm_planner::joint_plan::Request &req, xarm_planner::joint_plan::Response &res)
{
  group.setJointValueTarget(req.target);
  
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  res.success = success;
  ROS_INFO_NAMED("move_group_planner", "This plan (joint goal) %s", success ? "SUCCEEDED" : "FAILED");
  
  return success;
}

/* execution subscriber call-back function */
void XArmSimplePlanner::execute_plan_topic(const std_msgs::Bool::ConstPtr& exec)
{
  if(exec->data)
    group.move();
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "xarm_move_group_planner");

  XArmSimplePlanner planner;

  planner.start();

  ROS_INFO("Waiting for shutdown");

  /* necessary: because AsyncSpinner is not operating in the same thread */
  ros::waitForShutdown();
  return 0;
}

