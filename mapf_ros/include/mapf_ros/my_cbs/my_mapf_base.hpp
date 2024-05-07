/*
 * @Descripttion:
 * @version:
 * @Author: CyberC3
 * @Date: 2024-04-20 21:22:25
 * @LastEditors: zhu-hu
 * @LastEditTime: 2024-04-20 21:58:08
 */
#pragma once

#include "mapf_msgs/GlobalPlan.h"
#include "mapf_msgs/Goal.h"
#include "mapf_msgs/SinglePlan.h"
#include "mapf_ros/my_cbs/my_cbs_ros.hpp"
#include <ros/ros.h>

namespace mapf {
class MYMAPFBase {
public:
  MYMAPFBase();
  ~MYMAPFBase();

  void getParam();

  void goalCallback(const mapf_msgs::Goal::ConstPtr &goal);

  nav_msgs::Path getRobotPose();

  // check if reach goal
  bool reachGoal();

  void doMAPFThread();

  void stateMachine();

  void publishPlan(const mapf_msgs::GlobalPlan &plan);

private:
  std::mutex mtx_mapf_goal_;
  std::mutex mtx_planner_;

  ros::NodeHandle nh_;
  ros::Subscriber sub_mapf_goal_;
  ros::V_Publisher pub_gui_plan_;
  ros::Publisher pub_mapf_global_plan_;

  // mapf params
  std::string planner_name_;
  int agent_num_;
  std::string global_frame_id_;
  double planner_time_tolerance_;
  double goal_tolerance_;
  std::vector<std::string> base_frame_id_;
  std::vector<std::string> plan_topic_;

  nav_msgs::Path goal_ros_;
  bool receive_mapf_goal_;
  bool run_mapf_;

  boost::thread *do_mapf_thread_;
  boost::thread *state_machine_thread_;

  //   pluginlib::ClassLoader<mapf::MAPFROS> mapf_loader_;
  //   boost::shared_ptr<mapf::MAPFROS> mapf_planner_;
};

} // namespace mapf