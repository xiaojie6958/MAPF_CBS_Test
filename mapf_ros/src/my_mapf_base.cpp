#include "mapf_ros/my_cbs/my_mapf_base.hpp"

namespace mapf {

MYMAPFBase::MYMAPFBase()
    : nh_("~"), receive_mapf_goal_(false), run_mapf_(false) {

  getParam();

  pub_gui_plan_.resize(agent_num_);
  for (int i = 0; i < agent_num_; ++i) {
    pub_gui_plan_[i] = nh_.advertise<nav_msgs::Path>(plan_topic_[i], 1);
  }

  // goal in mapf form
  sub_mapf_goal_ = nh_.subscribe<mapf_msgs::Goal>(
      "mapf_goal", 1, &MYMAPFBase::goalCallback, this);
  pub_mapf_global_plan_ =
      nh_.advertise<mapf_msgs::GlobalPlan>("global_plan", 1);

  do_mapf_thread_ =
      new boost::thread(boost::bind(&MYMAPFBase::doMAPFThread, this));
  state_machine_thread_ =
      new boost::thread(boost::bind(&MYMAPFBase::stateMachine, this));
}

MYMAPFBase::~MYMAPFBase() {

  // then delete the thread
  do_mapf_thread_->interrupt();
  do_mapf_thread_->join();
  delete do_mapf_thread_;
  state_machine_thread_->interrupt();
  state_machine_thread_->join();
  delete state_machine_thread_;
}

void MYMAPFBase::getParam() {
  nh_.param<std::string>("mapf_planner", planner_name_, "mapf_planner/CBSROS");

  nh_.param<double>("planner_time_tolerance", planner_time_tolerance_, DBL_MAX);
  nh_.param<double>("goal_tolerance", goal_tolerance_, 1.0);
  nh_.param<std::string>("global_frame_id", global_frame_id_, "map");
  nh_.param<int>("agent_num", agent_num_, 1);

  base_frame_id_.resize(agent_num_);
  plan_topic_.resize(agent_num_);

  for (int i = 0; i < agent_num_; ++i) {
    nh_.param<std::string>("base_frame_id/agent_" + std::to_string(i),
                           base_frame_id_[i], "base_link");
    nh_.param<std::string>("plan_topic/agent_" + std::to_string(i),
                           plan_topic_[i], "plan");
  }
}

void MYMAPFBase::goalCallback(const mapf_msgs::Goal::ConstPtr &goal) {
  std::lock_guard<std::mutex> lock(mtx_mapf_goal_);
  goal_ros_ = goal->goal;
  goal_ros_.header.frame_id = global_frame_id_;
  goal_ros_.header.stamp = ros::Time::now();
  receive_mapf_goal_ = true;
}

nav_msgs::Path MYMAPFBase::getRobotPose() {
  nav_msgs::Path start;
  start.header.frame_id = global_frame_id_;
  start.header.stamp = ros::Time::now();
  start.poses.clear();
  start.poses.resize(agent_num_);

  for (int i = 0; i < agent_num_; ++i) {
    // // get tf
    // tf2::toMsg(tf2::Transform::getIdentity(), start.poses[i].pose);
    // geometry_msgs::PoseStamped robot_pose;
    // tf2::toMsg(tf2::Transform::getIdentity(), robot_pose.pose);
    // robot_pose.header.frame_id = base_frame_id_[i];
    // robot_pose.header.stamp = ros::Time();
    // tf_.transform(robot_pose, start.poses[i], global_frame_id_);
  }
  return start;
}

bool MYMAPFBase::reachGoal() {
  bool reach = true;
  nav_msgs::Path start = getRobotPose();
  for (int i = 0; i < start.poses.size(); ++i) {
    double diff_x = start.poses[i].pose.position.x -
                    goal_ros_.poses[i].pose.position.x,
           diff_y = start.poses[i].pose.position.y -
                    goal_ros_.poses[i].pose.position.y;
    reach &= (abs(diff_x) < goal_tolerance_ && abs(diff_y) < goal_tolerance_);
  }
  return reach;
}

void MYMAPFBase::stateMachine() {
  ROS_INFO_NAMED("MAPF state machine", "MAPF state machine Thread...");
  ros::NodeHandle thread_nh;
  ros::Rate loop_rate(5);

  try {
    while (thread_nh.ok()) {
      std::unique_lock<std::mutex> lock(mtx_mapf_goal_);
      std::unique_lock<std::mutex> lock_planner(mtx_planner_);

      if (receive_mapf_goal_) {
        receive_mapf_goal_ = false;
        run_mapf_ = true;
      }

      if (run_mapf_) {
        if (reachGoal()) {
          run_mapf_ = false;
        }
      }

      lock.unlock();
      lock_planner.unlock();
      loop_rate.sleep();
      boost::this_thread::interruption_point();
    } // end while
  } catch (...) {
    ROS_INFO("Exit State Machine thread.");
  }
}

void MYMAPFBase::doMAPFThread() {
  ROS_INFO_NAMED("MAPF thread", "Start active mapf algorithm...");
  ros::NodeHandle thread_nh;
  ros::Rate loop_rate(10);

  try {
    while (thread_nh.ok()) {
      std::unique_lock<std::mutex> lock_planner(mtx_planner_);
      bool run_mapf = run_mapf_;
      lock_planner.unlock();
      if (run_mapf) {
        nav_msgs::Path start_ros = getRobotPose();
        double cost = 0;
        mapf_msgs::GlobalPlan plan;
        if (mapf_planner_->makePlan(start_ros, goal_ros_, plan, cost,
                                    planner_time_tolerance_)) {
          publishPlan(plan);
        }
      }
      loop_rate.sleep();
      boost::this_thread::interruption_point();
    } // end while
  } catch (...) {
    ROS_INFO("Exit Do mapf thread.");
  }
}

void MYMAPFBase::publishPlan(const mapf_msgs::GlobalPlan &plan) {
  for (size_t i = 0; i < plan.global_plan.size(); ++i) {
    pub_gui_plan_[i].publish(plan.global_plan[i].plan);
  }
  pub_mapf_global_plan_.publish(plan);
}
} // namespace mapf