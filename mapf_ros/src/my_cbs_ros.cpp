/*
 * @Descripttion:
 * @version:
 * @Author: CyberC3
 * @Date: 2024-04-14 22:57:43
 * @LastEditors: zhu-hu
 * @LastEditTime: 2024-07-14 17:26:06
 */
#include <ros/ros.h>

#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>

#include "mapf_msgs/GlobalPlan.h"
#include "mapf_msgs/Goal.h"
#include "mapf_msgs/SinglePlan.h"

// ROS Wrapper for MYCBS
#include "mapf_ros/my_cbs/my_cbs_ros.hpp"

namespace mapf {

MYCBSROS::MYCBSROS(ros::NodeHandle *nh) : nh_(nh), initialized_(false) {
  map_parser_ = new map_parser::MapParser(nh_);

  map_parser_->parse();

  map_nodes_ = std::move(map_parser_->map_nodes());

  original_network_array_ = std::move(map_parser_->road_network());

  node_name_to_id_ = map_parser_->node_name_to_id();

  node_id_to_name_ = map_parser_->node_id_to_name();

  vehicle_init_ids_.clear();

  start_ids_.clear();

  goal_ids_.clear();

  nh_->param<bool>("real_car", real_car_, false);

  std::cout << "real car : " << real_car_ << std::endl;

  pub_start_goal_point_ =
      nh_->advertise<visualization_msgs::MarkerArray>("/start_gaol_points", 5);

  pub_start_goal_text_ =
      nh_->advertise<visualization_msgs::MarkerArray>("/start_goal_texts", 5);

  pub_one_step_pos_ =
      nh_->advertise<visualization_msgs::MarkerArray>("/one_step_pos", 5);

  sub_new_order_ =
      nh_->subscribe("/new_order", 5, &MYCBSROS::newOrderCallback, this);

  sub_order_from_web_ =
      nh_->subscribe("/setOrder", 5, &MYCBSROS::setOrderCallback, this);
}

MYCBSROS::MYCBSROS(std::string name) : initialized_(false) { initialize(name); }

void MYCBSROS::initialize(std::string name) {
  if (!initialized_) {
    // ROS_INFO("New CBS planner.");

    initialized_ = true;
  }
}

bool MYCBSROS::makePlan(const std::vector<Task> task,
                        mapf_msgs::GlobalPlan &plan, double &cost,
                        std::vector<std::vector<int>> &all_path_ids,
                        const double &time_tolerance) {
  // until tf can handle transforming things that are way in the past... we'll
  // require the goal to be in our global frame

  // garage是有车停靠的车库的位置
  if (task.empty()) {
    ROS_ERROR("garage or task vectors are empty!");
    return false;
  }

  std::vector<int> assign_result;
  std::vector<int> start;
  std::vector<int> goal_ids;
  std::vector<int> garage;
  for (int i = 0; i < task.size(); i++) {
    start.emplace_back(task[i].task_start_id);
    goal_ids.emplace_back(task[i].task_goal_id);
  }

  //////////////////////////////
  // for (int i = 0; i < garage_list_.size(); i++) {
  //   if (garage_list_[i].parking_vehicle_index.empty() == false) {
  //     garage.emplace_back(garage_list_[i].map_point_id);
  //   }
  // }
  ////////////////////////////////

  //先进行任务分配：
  // garage存储的是车辆点在地图中的位置索引下标
  // start存储的是起始点在地图中的位置索引下标
  // assign_result存储的是分配给start的车辆点在地图中的位置索引下标
  //////////
  garage.clear();
  for (int i = 0; i < vehicle_list_.size(); i++) {
    if (vehicle_list_[i].car_status == 2)
      continue;
    if (vehicle_list_[i].car_status == 1) {
      garage.emplace_back(
          garage_list_[vehicle_list_[i].garage_id].map_point_id);
    }
    if (vehicle_list_[i].car_status == 3) {
      int task_id = vehicle_list_[i].task_id;
      int vehicle_cur_id = findNextId(task_id);
      garage.emplace_back(vehicle_cur_id);
    }
  }
  ////////////
  assignTasks(garage, start, assign_result);

  /////////////////////////////////////////
  // for (int i = 0; i < start.size(); i++) {
  //   int garage_index = -1;
  //   for (int j = 0; j < garage_list_.size(); j++) {
  //     if (assign_result[i] == garage_list_[j].map_point_id) {
  //       garage_index = j;
  //       break;
  //     }
  //   }

  //   int car_index = garage_list_[garage_index].parking_vehicle_index.front();
  //   task_list_[task[i].task_id].execution_car_id = car_index;

  //   task_list_[task[i].task_id].task_status = 2;

  //   vehicle_list_[car_index].task_id = task[i].task_id;
  //   vehicle_list_[car_index].car_status = 2;

  //   garage_list_[garage_index].parking_vehicle_index.erase(
  //       garage_list_[garage_index].parking_vehicle_index.begin());
  // }
  ////////////////////////////////////////////

  ////////////////////////////
  for (int i = 0; i < start.size(); i++) {
    int garage_index = -1;
    for (int j = 0; j < garage_list_.size(); j++) {
      if (assign_result[i] == garage_list_[j].map_point_id) {
        garage_index = j;
        break;
      }
    }
    if (garage_index != -1) {
      int car_index = garage_list_[garage_index].parking_vehicle_index.front();
      task_list_[task[i].task_id].execution_car_id = car_index;

      task_list_[task[i].task_id].task_status = 2;

      vehicle_list_[car_index].task_id = task[i].task_id;
      vehicle_list_[car_index].car_status = 2;

      garage_list_[garage_index].parking_vehicle_index.erase(
          garage_list_[garage_index].parking_vehicle_index.begin());
    } else {
      int car_index = -1;
      for (int i = 0; i < vehicle_list_.size(); i++) {
        if (vehicle_list_[i].car_status == 3) {
          car_index = i;
          break;
        }
      }

      if (car_index != -1) {
        task_list_[vehicle_list_[car_index].task_id].task_status = 4;

        task_list_[task[i].task_id].execution_car_id = car_index;
        task_list_[task[i].task_id].task_status = 2;

        vehicle_list_[car_index].task_id = task[i].task_id;
        vehicle_list_[car_index].car_status = 2;
      } else {
        std::cout << "task assign error!!!" << std::endl;
      }
    }
  }

  ///////////////////////////
  //打印任务分配情况
  for (int i = 0; i < vehicle_list_.size(); i++) {
    std::cout << "vehicle[" << i << "] : " << vehicle_list_[i].task_id
              << std::endl;
  }

  int agent_num = assign_result.size();
  // start_ids_ = start;
  // goal_ids_ = goal;
  // mapf env
  std::vector<State> startStates;
  std::vector<Location> goals;

  //先求取车辆起始位置到相应任务起点的路径
  // get agents start pose in world frame
  for (int i = 0; i < agent_num; ++i) {
    // transform to map form
    startStates.emplace_back(State(0, 0, 0, assign_result[i]));

    goals.emplace_back(Location(0, 0, start[i]));

  } // end for

  // mapf search

  std::vector<PlanResult<State, Action, int>> solution;

  Environment mapf(goals, map_nodes_, node_name_to_id_, node_id_to_name_,
                   original_network_array_);
  MYCBS<State, Action, int, Conflict, Constraints, Environment> my_cbs(mapf);

  Timer timer;
  bool success = my_cbs.search(startStates, solution, time_tolerance);
  // check time tolerance
  timer.stop();
  if (timer.elapsedSeconds() > time_tolerance) {
    ROS_ERROR("Planning time out! Cur time tolerance is %lf", time_tolerance);
    return false;
  }

  if (success) {
    cost = 0;
    generatePlan(solution, goal_ids, plan, cost);

    all_path_ids.clear();
    for (int i = 0; i < solution.size(); ++i) {
      all_pos_switch_index_.emplace_back(solution[i].states.size());
      std::vector<int> path_id;
      path_id.clear();
      for (int j = 0; j < solution[i].states.size(); j++) {
        path_id.emplace_back(solution[i].states[j].first.id);
      }
      all_path_ids.emplace_back(path_id);
    }

    // all_path_ids_.clear();
    // all_path_ids_ = all_path_ids;

    // calculateAllPos();

    ROS_DEBUG_STREAM("Planning successful!");
    ROS_DEBUG_STREAM("runtime: " << timer.elapsedSeconds());
    ROS_DEBUG_STREAM("cost: " << cost);
    ROS_DEBUG_STREAM("makespan(involve start & end): " << plan.makespan);
  } else {
    ROS_ERROR("Planning NOT successful!");
  }

  //再求取车辆从相应任务起点到相应任务终点的路径

  startStates.clear();

  goals.clear();

  for (int i = 0; i < start.size(); ++i) {
    startStates.emplace_back(State(0, 0, 0, start[i]));

    goals.emplace_back(Location(0, 0, goal_ids[i]));
  }

  std::vector<PlanResult<State, Action, int>> solution2;

  Environment mapf2(goals, map_nodes_, node_name_to_id_, node_id_to_name_,
                    original_network_array_);
  MYCBS<State, Action, int, Conflict, Constraints, Environment> my_cbs2(mapf2);
  Timer timer2;

  bool success2 = my_cbs2.search(startStates, solution2, time_tolerance);

  timer2.stop();

  if (timer2.elapsedSeconds() > time_tolerance) {
    ROS_ERROR("Planning time out! Cur time tolerance is %lf", time_tolerance);
    return false;
  }

  if (success2) {
    cost = 0;
    generatePlan(solution2, goal_ids, plan, cost);

    for (int i = 0; i < solution2.size(); ++i) {
      std::vector<int> path_id;
      path_id.clear();
      for (int j = 1; j < solution2[i].states.size(); j++) {
        path_id.emplace_back(solution2[i].states[j].first.id);

        all_path_ids[i].emplace_back(solution2[i].states[j].first.id);
        // all_path_ids_[i].emplace_back(solution2[i].states[j].first.id);
      }
      // all_path_ids.emplace_back(path_id);
      // all_path_ids_.emplace_back(path_id);

      all_path_ids_.emplace_back(all_path_ids[i]);

      ROS_DEBUG_STREAM("Planning successful!");
      ROS_DEBUG_STREAM("runtime: " << timer2.elapsedSeconds());
      ROS_DEBUG_STREAM("cost: " << cost);
      ROS_DEBUG_STREAM("makespan(involve start & end): " << plan.makespan);
    }
  } else {
    ROS_ERROR("Planning 2 NOT successful!");
  }

  return (success && success2);
}

bool MYCBSROS::makePlan(const Task task, mapf_msgs::GlobalPlan &plan,
                        double &cost,
                        std::vector<std::vector<int>> &all_path_ids,
                        const double &time_tolerance) {
  std::vector<State> startStates;
  std::vector<Location> goals;

  //先求取车辆起始位置到相应任务起点的路径
  // get agents start pose in world frame

  // transform to map form
  startStates.emplace_back(State(0, 0, 0, task.task_start_id));

  goals.emplace_back(Location(0, 0, task.task_goal_id));

  std::vector<int> goal_id;
  goal_id.emplace_back(task.task_goal_id);

  // mapf search

  std::vector<PlanResult<State, Action, int>> solution;

  Environment mapf(goals, map_nodes_, node_name_to_id_, node_id_to_name_,
                   original_network_array_);
  MYCBS<State, Action, int, Conflict, Constraints, Environment> my_cbs(mapf);

  Timer timer;
  bool success = my_cbs.search(startStates, solution, time_tolerance);
  // check time tolerance
  timer.stop();
  if (timer.elapsedSeconds() > time_tolerance) {
    ROS_ERROR("Planning time out! Cur time tolerance is %lf", time_tolerance);
    return false;
  }

  if (success) {
    cost = 0;
    generatePlan(solution, goal_id, plan, cost);

    all_path_ids.clear();
    for (int i = 0; i < solution.size(); ++i) {
      all_pos_switch_index_.emplace_back(solution[i].states.size());
      std::vector<int> path_id;
      path_id.clear();
      for (int j = 0; j < solution[i].states.size(); j++) {
        path_id.emplace_back(solution[i].states[j].first.id);
      }
      all_path_ids.emplace_back(path_id);
    }

    // all_path_ids_.clear();
    // all_path_ids_ = all_path_ids;

    // calculateAllPos();

    ROS_DEBUG_STREAM("Planning successful!");
    ROS_DEBUG_STREAM("runtime: " << timer.elapsedSeconds());
    ROS_DEBUG_STREAM("cost: " << cost);
    ROS_DEBUG_STREAM("makespan(involve start & end): " << plan.makespan);
  } else {
    ROS_ERROR("Planning NOT successful!");
  }
}

bool MYCBSROS::makePlan(mapf_msgs::GlobalPlan &plan, double &cost,
                        std::vector<std::vector<int>> &all_path_ids,
                        const double &time_tolerance, bool real_car) {
  if (start_ids_.empty() || goal_ids_.empty()) {
    ROS_ERROR("Start or goal vectors are empty!");
    return false;
  }
  if (start_ids_.size() != goal_ids_.size()) {
    ROS_ERROR("Start and goal vectors are not the same length!");
    return false;
  }

  int agent_num = start_ids_.size();

  // mapf env
  std::vector<State> startStates;
  std::vector<Location> goals;

  //先求取车辆起始位置到相应任务起点的路径
  // get agents start pose in world frame
  for (int i = 0; i < agent_num; ++i) {
    // transform to map form
    startStates.emplace_back(State(0, 0, 0, start_ids_[i]));

    goals.emplace_back(Location(0, 0, goal_ids_[i]));

  } // end for

  // mapf search

  std::vector<PlanResult<State, Action, int>> solution;

  Environment mapf(goals, map_nodes_, node_name_to_id_, node_id_to_name_,
                   original_network_array_);
  MYCBS<State, Action, int, Conflict, Constraints, Environment> my_cbs(mapf);

  Timer timer;
  bool success = my_cbs.search(startStates, solution, time_tolerance);
  // check time tolerance
  timer.stop();
  if (timer.elapsedSeconds() > time_tolerance) {
    ROS_ERROR("Planning time out! Cur time tolerance is %lf", time_tolerance);
    return false;
  }

  if (success) {
    cost = 0;
    generatePlan(solution, goal_ids_, plan, cost);

    all_path_ids.clear();
    for (int i = 0; i < solution.size(); ++i) {
      all_pos_switch_index_.emplace_back(solution[i].states.size());
      std::vector<int> path_id;
      path_id.clear();
      for (int j = 0; j < solution[i].states.size(); j++) {
        path_id.emplace_back(solution[i].states[j].first.id);
      }
      all_path_ids.emplace_back(path_id);
    }

    all_path_ids_.clear();
    all_path_ids_ = all_path_ids;

    // calculateAllPos();

    ROS_DEBUG_STREAM("Planning successful!");
    ROS_DEBUG_STREAM("runtime: " << timer.elapsedSeconds());
    ROS_DEBUG_STREAM("cost: " << cost);
    ROS_DEBUG_STREAM("makespan(involve start & end): " << plan.makespan);
  } else {
    ROS_ERROR("Planning NOT successful!");
  }

  return success;
}

void MYCBSROS::generatePlan(
    const std::vector<PlanResult<State, Action, int>> &solution,
    const std::vector<int> &goal, mapf_msgs::GlobalPlan &plan, double &cost) {
  int &makespan = plan.makespan;
  for (const auto &s : solution) {
    cost += s.cost;
    makespan = std::max<int>(makespan, s.cost);
  }
  // add start point (the fisrt step is to get the center of the first grid)
  makespan += 1;

  plan.global_plan.resize(solution.size());

  std::cout << "solution_size : " << solution.size() << std::endl;

  for (size_t i = 0; i < solution.size(); ++i) {
    // create a message for the plan
    mapf_msgs::SinglePlan &single_plan = plan.global_plan[i];
    nav_msgs::Path &single_path = single_plan.plan;
    single_path.header.frame_id = "world";
    single_path.header.stamp = ros::Time::now();
    std::cout << "result[" << i << "] : ";

    for (const auto &state : solution[i].states) {
      geometry_msgs::PoseStamped cur_pose;
      cur_pose.header.frame_id = single_path.header.frame_id;
      cur_pose.pose.orientation.w = 1;
      cur_pose.pose.position.x = state.first.x;
      cur_pose.pose.position.y = state.first.y;
      single_path.poses.push_back(cur_pose);
      single_plan.time_step.push_back(state.second);
      std::cout << state.first.id << ", ";
    }
    std::cout << std::endl;

    // replace end point with goal point
    // single_path.poses.back() = goal.poses[i];

  } // end solution for
}

// void MYCBSROS::calculateAllPos() {
//   all_pos_.clear();
//   std::vector<std::pair<double, double>> one_path;
//   double step_length = 0.2;
//   for (int i = 0; i < all_path_ids_.size(); i++) {
//     one_path.clear();
//     for (int j = 1; j < all_path_ids_[i].size(); j++) {
//       one_path.emplace_back(
//           std::make_pair(map_nodes_[all_path_ids_[i][j - 1]].x_pos,
//                          map_nodes_[all_path_ids_[i][j - 1]].y_pos));
//       double length = std::hypot(map_nodes_[all_path_ids_[i][j]].x_pos -
//                                      map_nodes_[all_path_ids_[i][j -
//                                      1]].x_pos,
//                                  map_nodes_[all_path_ids_[i][j]].y_pos -
//                                      map_nodes_[all_path_ids_[i][j -
//                                      1]].y_pos);
//       int expand_step = std::ceil(length / step_length);
//       std::pair<double, double> unit{
//           (map_nodes_[all_path_ids_[i][j]].x_pos -
//            map_nodes_[all_path_ids_[i][j - 1]].x_pos) /
//               length,
//           (map_nodes_[all_path_ids_[i][j]].y_pos -
//            map_nodes_[all_path_ids_[i][j - 1]].y_pos) /
//               length};
//       for (int k = 1; k < expand_step; k++) {
//         one_path.emplace_back(
//             std::make_pair(map_nodes_[all_path_ids_[i][j - 1]].x_pos +
//                                k * unit.first * step_length,
//                            map_nodes_[all_path_ids_[i][j - 1]].y_pos +
//                                k * unit.second * step_length));
//       }
//     }
//     one_path.emplace_back(
//         std::make_pair(map_nodes_[all_path_ids_[i].back()].x_pos,
//                        map_nodes_[all_path_ids_[i].back()].y_pos));
//     std::cout << "one path[" << i << "] nums : " << one_path.size()
//               << std::endl;
//     all_pos_.emplace_back(one_path);
//   }
// }

void MYCBSROS::newOrderCallback(const std_msgs::BoolConstPtr &msg_in) {
  //第一步：保存现有的all_results_;
  auto original_all_results = all_results_;

  //新订单来临之后，存储原有机器人的新起点
  std::vector<ResultPoint> original_new_start_points;
  original_new_start_points.clear();
  original_new_start_points.resize(all_results_.size());

  std::vector<int> original_next_index;
  original_next_index.resize(all_results_.size());
  int point_count = map_nodes_.size();

  for (int i = 0; i < all_results_.size(); i++) {
    if (control_step_ - 1 >= all_pos_[i].size()) {
      ResultPoint pt;
      pt.cost = 0;
      pt.wait_time = 0;
      pt.point_id = -1;
      pt.x_y = std::make_pair(-1, -1);
      original_new_start_points[i] = pt;
      original_next_index[i] = -1;
      continue;
    }
    ResultPoint pt;
    pt.cost = 0;
    pt.wait_time = 0;
    pt.x_y = all_pos_[i][control_step_ - 1];
    pt.point_id = point_count++;
    original_new_start_points[i] = pt;
    for (int j = 0; j < all_results_[i].size() - 1; j++) {
      if (control_step_ - 1 > all_results_[i][j].cost) {
        original_next_index[i] = j + 1;
      }
    }
  }

  original_all_results.clear();
  for (int i = 0; i < all_results_.size(); i++) {
    if (original_next_index[i] == -1)
      continue;

    std::vector<ResultPoint> one_path;
    one_path.clear();
    one_path.emplace_back(original_new_start_points[i]);
    one_path.insert(one_path.end(),
                    all_results_[i].begin() + original_next_index[i],
                    all_results_[i].end());
    original_all_results.emplace_back(one_path);
  }

  //还需要更新original_all_results里的cost!!!
  for (int i = 0; i < original_all_results.size(); i++) {
    for (int j = 1; j < original_all_results[i].size(); j++) {
      double length = std::hypot(original_all_results[i][j].x_y.first -
                                     original_all_results[i][j - 1].x_y.first,
                                 original_all_results[i][j].x_y.second -
                                     original_all_results[i][j - 1].x_y.second);
      int expand_step = std::ceil(length / 0.2);
      original_all_results[i][j].cost =
          original_all_results[i][j - 1].cost + expand_step;
    }
  }

  //第二步：将已有的路径截掉走过的路径，重新保存all_path_ids_

  //第三步：重新规划新传进来的任务

  //第四步：将原来的all_path_ids_存入新的all_path_ids_

  mapf_msgs::GlobalPlan plan;
  std::vector<std::vector<int>> all_path_ids;
  double cost = 0;
  double time_tolerance = 5.0;

  //新订单测试
  std::vector<int> start{55, 38};
  std::vector<int> goal{40, 13};

  std::vector<Task> tasks;

  tasks.emplace_back();
  tasks.back().task_id = task_list_.size();
  tasks.back().task_status = 1;
  tasks.back().task_start_id = start[0];
  tasks.back().task_goal_id = goal[0];
  tasks.emplace_back();
  tasks.back().task_id = task_list_.size();
  tasks.back().task_status = 1;
  tasks.back().task_start_id = start[1];
  tasks.back().task_goal_id = goal[1];

  makePlan(tasks, plan, cost, all_path_ids, time_tolerance);

  std::cout << "Plan result : " << std::endl;

  generateAllResult(all_path_ids, 0.2);

  //把两个部分合并
  all_results_.insert(all_results_.begin(), original_all_results.begin(),
                      original_all_results.end());

  generateAllRoutePathForShow();

  //第五步：进行冲突检测及冲突消解
  mapf::MYCBSROS::Conflict conflict;
  while (firstConflictDetect(conflict) == true) {

    conflictSolve(conflict);
  }

  printAllResult();

  //第六步：重新计算新的all_pos_
  calculateAllPos();

  control_step_ = 0;
}

void MYCBSROS::setOrderCallback(const std_msgs::StringConstPtr &msg_in) {

  std::istringstream is(msg_in->data);
  boost::property_tree::ptree pt;
  boost::property_tree::read_json(is, pt);

  auto s = pt.get<std::string>("end");
  //去掉中括号:"[]"
  s.erase(s.begin());
  s.erase(s.end() - 1);
  //去掉大括号："{}"
  s.erase(std::remove(s.begin(), s.end(), '{'), s.end());
  s.erase(std::remove(s.begin(), s.end(), '}'), s.end());

  std::cout << "s:" << s << std::endl;

  std::istringstream iss(s);
  std::vector<int> starts;
  std::vector<int> goals;
  std::string token;
  int count = 0;

  while (std::getline(iss, token, ',')) {
    std::istringstream tokenStream(token);
    int value;
    if (tokenStream >> value) {
      if (count % 2 == 0)
        starts.push_back(value);
      else
        goals.push_back(value);

      count++;
    } else {
      ROS_ERROR_STREAM("Failed to parse integer from token: " << token);
    }
  }

  //订单触发调度任务，首次触发和非首次触发处理不同
  if (car_running_ == true) {
    newOrderComeIn(starts, goals);
  } else {
    firstOrderComeIn(starts, goals);
    car_running_ = true;
  }
}

void MYCBSROS::calculateAllPos() {
  all_pos_.clear();
  std::vector<std::pair<double, double>> one_path;
  double step_length = 0.2;
  std::cout << "all result size : " << all_results_.size() << std::endl;
  for (int i = 0; i < all_results_.size(); i++) {
    //如果任务已经执行完了，则不需要再计算空的路径点了
    if (task_list_[i].task_status == 4) {
      all_pos_.emplace_back();
      continue;
    }
    one_path.clear();
    std::cout << "all result size[" << i << "] : " << all_results_[i].size()
              << std::endl;
    for (int j = 1; j < all_results_[i].size(); j++) {
      one_path.emplace_back(all_results_[i][j - 1].x_y);
      int wait_time = all_results_[i][j - 1].wait_time;
      while (wait_time > 0) {
        one_path.emplace_back(all_results_[i][j - 1].x_y);
        wait_time--;
      }
      double length = std::hypot(
          all_results_[i][j].x_y.first - all_results_[i][j - 1].x_y.first,
          all_results_[i][j].x_y.second - all_results_[i][j - 1].x_y.second);
      int expand_step = std::ceil(length / step_length);
      std::pair<double, double> unit{
          (all_results_[i][j].x_y.first - all_results_[i][j - 1].x_y.first) /
              length,
          (all_results_[i][j].x_y.second - all_results_[i][j - 1].x_y.second) /
              length};
      for (int k = 1; k < expand_step; k++) {
        one_path.emplace_back(std::make_pair(
            all_results_[i][j - 1].x_y.first + k * unit.first * step_length,
            all_results_[i][j - 1].x_y.second + k * unit.second * step_length));
      }
      if (j == all_pos_switch_index_[i] - 1) {
        final_switch_index_.emplace_back(one_path.size());
      }
    }
    one_path.emplace_back(all_results_[i].back().x_y);
    std::cout << "one path[" << i << "] nums : " << one_path.size()
              << std::endl;
    all_pos_.emplace_back(one_path);
  }

  for (int i = 0; i < all_pos_.size(); i++) {
    std::cout << "all_pos[" << i << "] : " << all_pos_[i].size() << std::endl;
  }
}

void MYCBSROS::publishOneStepPos(const int step) {
  int count = 0;
  int loop_step = step;
  one_step_pos_.markers.clear();
  visualization_msgs::Marker point_marker;
  visualization_msgs::Marker text_marker;
  point_marker.header.frame_id = "world";
  point_marker.header.stamp = ros::Time::now();
  point_marker.ns = "all_points";
  point_marker.action = visualization_msgs::Marker::ADD;
  point_marker.lifetime = ros::Duration(0);
  point_marker.type = visualization_msgs::Marker::CYLINDER;
  point_marker.scale.x = 1.5;
  point_marker.scale.y = 1.5;
  point_marker.scale.z = 1.5;

  text_marker.header.frame_id = "world";
  text_marker.header.stamp = ros::Time::now();
  text_marker.ns = "all_points";
  text_marker.action = visualization_msgs::Marker::ADD;
  text_marker.lifetime = ros::Duration(0);
  text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  text_marker.scale.x = 1.5;
  text_marker.scale.y = 1.5;
  text_marker.scale.z = 1.5;
  text_marker.color.r = 0.0;
  text_marker.color.g = 0.0;
  text_marker.color.b = 0.0;
  text_marker.color.a = 1.0;

  for (int i = 0; i < vehicle_list_.size(); i++) {
    point_marker.header.seq = count;
    point_marker.id = count;
    if (count % 3 == 0) {
      point_marker.color.r = 1.0;
      point_marker.color.g = 0.0;
      point_marker.color.b = 0.0;
      point_marker.color.a = 1.0;
    } else if (count % 3 == 1) {
      point_marker.color.r = 0.0;
      point_marker.color.g = 0.0;
      point_marker.color.b = 1.0;
      point_marker.color.a = 1.0;
    } else {
      point_marker.color.r = 0.0;
      point_marker.color.g = 1.0;
      point_marker.color.b = 1.0;
      point_marker.color.a = 1.0;
    }
    point_marker.color.a = 0.8;
    if (vehicle_list_[i].car_status == 1) {
      // int car_num =
      //     garage_list_[vehicle_list_[i].garage_id].parking_vehicle_index.size();
      // const auto &vec =
      //     garage_list_[vehicle_list_[i].garage_id].parking_vehicle_index;
      // int car_index = std::find(vec.begin(), vec.end(), i) - vec.begin();
      point_marker.pose.position.x =
          map_nodes_[garage_list_[vehicle_list_[i].garage_id].map_point_id]
              .x_pos;
      point_marker.pose.position.y =
          map_nodes_[garage_list_[vehicle_list_[i].garage_id].map_point_id]
              .y_pos;
    } else {
      int index = vehicle_list_[i].task_id;
      if (loop_step >= all_pos_[index].size()) {
        point_marker.pose.position.x = all_pos_[index].back().first;
        point_marker.pose.position.y = all_pos_[index].back().second;
        if (task_list_[index].task_status == 2) {
          std::cout << "loop_step : " << loop_step << std::endl;
          std::cout << "all_pos_[" << index
                    << "] size : " << all_pos_[index].size() << std::endl;
          std::cout << "vehicle id : " << i << std::endl;
          std::cout << "task id : " << index << std::endl;
          backToGarage(index);
          loop_step = 0;
        } else if (task_list_[index].task_status == 3) {

          std::cout << "task_status : 4" << std::endl;
          vehicle_list_[i].car_status = 1;
          task_list_[index].task_status = 4;
        }

      } else {
        point_marker.pose.position.x = all_pos_[index][loop_step].first;
        point_marker.pose.position.y = all_pos_[index][loop_step].second;
      }
    }

    count++;
    one_step_pos_.markers.emplace_back(point_marker);

    text_marker.header.seq = count;
    text_marker.id = count;

    if (vehicle_list_[i].car_status == 1) {
      text_marker.pose.position.x =
          map_nodes_[garage_list_[vehicle_list_[i].garage_id].map_point_id]
              .x_pos;
      text_marker.pose.position.y =
          map_nodes_[garage_list_[vehicle_list_[i].garage_id].map_point_id]
              .y_pos;
      text_marker.pose.position.z = 0.0;
    } else {
      int index = vehicle_list_[i].task_id;
      if (loop_step >= all_pos_[index].size()) {
        text_marker.pose.position.x = all_pos_[index].back().first;
        text_marker.pose.position.y = all_pos_[index].back().second;
        text_marker.pose.position.z = 0.0;

      } else {
        text_marker.pose.position.x = all_pos_[index][loop_step].first;
        text_marker.pose.position.y = all_pos_[index][loop_step].second;
        text_marker.pose.position.z = 0.0;
      }
    }

    text_marker.text = "V" + std::to_string(i);

    // if (real_car_ == false) {
    //   if (loop_step == -1) {
    //     text_marker.text = "V" + std::to_string(i);
    //   } else
    //     text_marker.text = "V" + std::to_string(i);
    // } else {
    //   text_marker.text = "V" + std::to_string(i);
    // }

    count++;

    one_step_pos_.markers.emplace_back(text_marker);
  }

  pub_one_step_pos_.publish(one_step_pos_);
}

void MYCBSROS::publishStartGoalPoint() {
  int count = 0;
  start_goal_points_.markers.clear();
  visualization_msgs::Marker point_marker;
  point_marker.header.frame_id = "world";
  point_marker.header.stamp = ros::Time::now();
  point_marker.ns = "start_goal_points";
  point_marker.action = visualization_msgs::Marker::ADD;
  point_marker.lifetime = ros::Duration(0.5);
  point_marker.type = visualization_msgs::Marker::CYLINDER;
  point_marker.scale.x = 2.0;
  point_marker.scale.y = 2.0;
  point_marker.scale.z = 2.0;
  for (int i = 0; i < task_list_.size(); ++i) {
    if (task_list_[i].task_status == 4)
      continue;
    point_marker.header.seq = count;
    point_marker.id = count;
    if (count % 3 == 0) {
      point_marker.color.r = 1.0;
      point_marker.color.g = 0.0;
      point_marker.color.b = 0.0;
      point_marker.color.a = 1.0;
    } else if (count % 3 == 1) {
      point_marker.color.r = 0.0;
      point_marker.color.g = 0.0;
      point_marker.color.b = 1.0;
      point_marker.color.a = 1.0;
    } else {
      point_marker.color.r = 0.0;
      point_marker.color.g = 1.0;
      point_marker.color.b = 1.0;
      point_marker.color.a = 1.0;
    }
    point_marker.color.a = 0.6;
    // point_marker.pose.position.x = all_results_[i].front().x_y.first;
    // point_marker.pose.position.y = all_results_[i].front().x_y.second;
    point_marker.pose.position.x =
        map_nodes_[task_list_[i].task_start_id].x_pos;
    point_marker.pose.position.y =
        map_nodes_[task_list_[i].task_start_id].y_pos;
    count++;

    start_goal_points_.markers.emplace_back(point_marker);
  }

  int color_count = 0;
  point_marker.type = visualization_msgs::Marker::CUBE;
  for (int i = 0; i < task_list_.size(); ++i) {
    if (task_list_[i].task_status == 4)
      continue;
    point_marker.header.seq = count;
    point_marker.id = count;
    if (color_count % 3 == 0) {
      point_marker.color.r = 1.0;
      point_marker.color.g = 0.0;
      point_marker.color.b = 0.0;
      point_marker.color.a = 1.0;
    } else if (color_count % 3 == 1) {
      point_marker.color.r = 0.0;
      point_marker.color.g = 0.0;
      point_marker.color.b = 1.0;
      point_marker.color.a = 1.0;
    } else {
      point_marker.color.r = 0.0;
      point_marker.color.g = 1.0;
      point_marker.color.b = 1.0;
      point_marker.color.a = 1.0;
    }
    point_marker.color.a = 0.6;
    // point_marker.pose.position.x = all_results_[i].back().x_y.first;
    // point_marker.pose.position.y = all_results_[i].back().x_y.second;
    point_marker.pose.position.x = map_nodes_[task_list_[i].task_goal_id].x_pos;
    point_marker.pose.position.y = map_nodes_[task_list_[i].task_goal_id].y_pos;
    count++;
    color_count++;

    start_goal_points_.markers.emplace_back(point_marker);
  }

  pub_start_goal_point_.publish(start_goal_points_);

  int id_count = 0;
  start_goal_texts_.markers.clear();
  visualization_msgs::Marker text_marker;
  text_marker.header.frame_id = "world";
  text_marker.header.stamp = ros::Time::now();
  text_marker.ns = "start_goal_text";
  text_marker.action = visualization_msgs::Marker::ADD;
  text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  text_marker.lifetime = ros::Duration(0.5);
  // Scale
  text_marker.scale.x = 2.0;
  text_marker.scale.y = 2.0;
  text_marker.scale.z = 2.0;
  // Color
  text_marker.color.r = 0.0;
  text_marker.color.g = 0.0;
  text_marker.color.b = 0.0;
  text_marker.color.a = 1.0;
  for (int i = 0; i < task_list_.size(); i++) {
    if (task_list_[i].task_status == 4)
      continue;
    text_marker.id = id_count++;
    // text_marker.pose.position.x = all_results_[i].front().x_y.first;
    // text_marker.pose.position.y = all_results_[i].front().x_y.second;
    text_marker.pose.position.x = map_nodes_[task_list_[i].task_start_id].x_pos;
    text_marker.pose.position.y = map_nodes_[task_list_[i].task_start_id].y_pos;
    text_marker.pose.position.z = 0.0;
    text_marker.text = "S" + std::to_string(i);
    start_goal_texts_.markers.emplace_back(text_marker);
  }

  for (int i = 0; i < task_list_.size(); i++) {
    if (task_list_[i].task_status == 4)
      continue;
    text_marker.id = id_count++;
    // text_marker.pose.position.x = all_results_[i].back().x_y.first;
    // text_marker.pose.position.y = all_results_[i].back().x_y.second;
    text_marker.pose.position.x = map_nodes_[task_list_[i].task_goal_id].x_pos;
    text_marker.pose.position.y = map_nodes_[task_list_[i].task_goal_id].y_pos;
    text_marker.pose.position.z = 0.0;
    text_marker.text = "G" + std::to_string(i);
    start_goal_texts_.markers.emplace_back(text_marker);
  }

  pub_start_goal_text_.publish(start_goal_texts_);
}

void MYCBSROS::generateAllResult(
    const std::vector<std::vector<int>> all_path_ids,
    const double step_length) {
  all_results_.clear();
  for (int i = 0; i < all_path_ids.size(); ++i) {
    std::vector<ResultPoint> one_path;
    one_path.clear();
    one_path.resize(all_path_ids[i].size());
    ResultPoint pt;
    pt.point_id = all_path_ids[i][0];
    pt.cost = 0;
    pt.wait_time = 0;
    pt.x_y = std::make_pair(map_nodes_[pt.point_id].x_pos,
                            map_nodes_[pt.point_id].y_pos);
    one_path[0] = pt;
    for (int j = 1; j < all_path_ids[i].size(); ++j) {
      pt.point_id = all_path_ids[i][j];
      double length = std::hypot(map_nodes_[all_path_ids[i][j]].x_pos -
                                     map_nodes_[all_path_ids[i][j - 1]].x_pos,
                                 map_nodes_[all_path_ids[i][j]].y_pos -
                                     map_nodes_[all_path_ids[i][j - 1]].y_pos);
      int expand_step = std::ceil(length / step_length);
      pt.cost = one_path[j - 1].cost + expand_step;
      pt.x_y = std::make_pair(map_nodes_[pt.point_id].x_pos,
                              map_nodes_[pt.point_id].y_pos);
      one_path[j] = pt;
    }
    all_results_.emplace_back(one_path);
  }

  generateAllRoutePathForShow();

  for (int i = 0; i < all_results_.size(); ++i) {
    for (int j = 0; j < all_results_[i].size(); ++j) {
      std::cout << "[" << all_results_[i][j].point_id << ", "
                << all_results_[i][j].cost << ", "
                << all_results_[i][j].wait_time << "], ";
    }
    std::cout << std::endl;
  }
}

void MYCBSROS::generateAllRoutePathForShow() {

  all_route_path_for_show_.clear();
  for (int i = 0; i < all_results_.size(); i++) {
    std::vector<std::pair<double, double>> one_path;
    one_path.clear();
    for (int j = 0; j < all_results_[i].size(); j++) {
      one_path.emplace_back(all_results_[i][j].x_y);
    }
    all_route_path_for_show_.emplace_back(one_path);
  }
}

void MYCBSROS::updateAllRoutePathForShow() {
  all_route_path_for_show_.clear();
  for (int i = 0; i < all_results_.size(); i++) {
    if (task_list_[i].task_status == 4)
      continue;
    int start_index = 0;
    int end_index = all_results_[i].size();
    // if (control_step_ < final_switch_index_[i]) {
    //   end_index = all_pos_switch_index_[i];
    // } else {
    //   start_index = all_pos_switch_index_[i] - 1;
    // }
    std::vector<std::pair<double, double>> one_path;
    one_path.clear();
    for (int j = start_index; j < end_index; j++) {
      one_path.emplace_back(all_results_[i][j].x_y);
    }
    all_route_path_for_show_.emplace_back(one_path);
  }
}

// conflict是输入
bool MYCBSROS::conflictSolve(Conflict conflict) {

  int path_i = conflict.conflict_id.first;
  int path_j = conflict.conflict_id.second;
  int point_i = conflict.point_ids.first;
  int point_j = conflict.point_ids.second;

  //处理终点冲突
  if (point_i == all_results_[path_i].size() - 1) {
    int i = point_i;
    int j = point_j;
    while (all_results_[path_i][i].point_id ==
           all_results_[path_j][j].point_id) {
      i--;
      j--;
    }
    // i这里要等
    all_results_[path_i][i].wait_time +=
        all_results_[path_j][point_j].cost +
        all_results_[path_j][point_j].wait_time -
        all_results_[path_i][point_i].cost + 15;
    //重新计算cost
    for (int k = i + 1; k < all_results_[path_i].size(); k++) {
      all_results_[path_i][k].cost += all_results_[path_i][i].wait_time;
    }

    return true;
  }

  //处理终点冲突
  if (point_j == all_results_[path_j].size() - 1) {
    std::cout << "c   ";
    int i = point_i;
    int j = point_j;
    while (all_results_[path_i][i].point_id ==
           all_results_[path_j][j].point_id) {
      i--;
      j--;
    }
    std::cout << "cc   ";
    all_results_[path_j][j].wait_time +=
        all_results_[path_i][point_i].cost +
        all_results_[path_i][point_i].wait_time -
        all_results_[path_j][point_j].cost + 15;
    std::cout << "ccc   ";
    //重新计算cost
    for (int k = j + 1; k < all_results_[path_j].size(); k++) {
      all_results_[path_j][k].cost += all_results_[path_j][j].wait_time;
    }
    std::cout << "cccc   ";
    return true;
  }

  //处理中间位置的冲突，选择短的那一条路径等待
  if (all_results_[path_i].back().cost < all_results_[path_j].back().cost) {
    all_results_[path_i][point_i - 1].wait_time +=
        all_results_[path_j][point_j].cost +
        all_results_[path_j][point_j].wait_time -
        all_results_[path_i][point_i].cost + 15;

    //重新计算后面点的cost
    for (int i = point_i; i < all_results_[path_i].size(); ++i) {
      all_results_[path_i][i].cost +=
          all_results_[path_i][point_i - 1].wait_time;
    }

    return true;
  } else {
    all_results_[path_j][point_j - 1].wait_time +=
        all_results_[path_i][point_i].cost +
        all_results_[path_i][point_i].wait_time -
        all_results_[path_j][point_j].cost + 15;

    //重新计算后面点的cost
    for (int i = point_j; i < all_results_[path_j].size(); ++i) {
      all_results_[path_j][i].cost +=
          all_results_[path_j][point_j - 1].wait_time;
    }

    return true;
  }
}

// conflict是输出
bool MYCBSROS::firstConflictDetect(Conflict &conflict) {
  // for (int i = 0; i < all_path_ids_.size(); ++i) {
  //   for (int j = i + 1; j < all_path_ids_.size(); ++j) {
  //     if (conflictDetect(i, j, conflict) == true) {
  //       //产生冲突
  //       return true;
  //     }
  //   }
  // }
  for (int i = 0; i < all_results_.size(); ++i) {
    for (int j = i + 1; j < all_results_.size(); ++j) {
      if (conflictDetect(i, j, conflict) == true) {
        //产生冲突
        return true;
      }
    }
  }
  //没有冲突了
  return false;
}

bool MYCBSROS::conflictDetect(int path_i, int path_j, Conflict &conflict) {
  for (int i = 0; i < all_results_[path_i].size(); ++i) {
    for (int j = 0; j < all_results_[path_j].size(); ++j) {
      //对中间的点进行冲突检查
      if (twoResultPointConflict(all_results_[path_i][i],
                                 all_results_[path_j][j])) {
        conflict.conflict_id = std::make_pair(path_i, path_j);
        conflict.point_ids = std::make_pair(i, j);
        std::cout << "check 0   ";
        return true;
      }
      //对终点冲突进行额外的检查
      if (i == all_results_[path_i].size() - 1 &&
          all_results_[path_i][i].point_id ==
              all_results_[path_j][j].point_id) {
        if (all_results_[path_i].back().cost <= all_results_[path_j][j].cost) {
          conflict.conflict_id = std::make_pair(path_i, path_j);
          conflict.point_ids = std::make_pair(i, j);
          std::cout << "check1   ";
          return true;
        }
      }
      //对终点冲突进行额外的检查
      if (j == all_results_[path_j].size() - 1 &&
          all_results_[path_j][j].point_id ==
              all_results_[path_i][i].point_id) {
        if (all_results_[path_j].back().cost <= all_results_[path_i][i].cost) {
          conflict.conflict_id = std::make_pair(path_i, path_j);
          conflict.point_ids = std::make_pair(i, j);
          std::cout << "check2   ";
          return true;
        }
      }
    }
  }

  return false;
}

bool MYCBSROS::twoResultPointConflict(const ResultPoint pt1,
                                      const ResultPoint pt2) {
  if (pt1.point_id != pt2.point_id)
    return false;

  int pt1_cost_low = pt1.cost - 5;
  int pt1_cost_high = pt1.cost + pt1.wait_time + 5;

  int pt2_cost_low = pt2.cost - 5;
  int pt2_cost_high = pt2.cost + pt2.wait_time + 5;

  if (pt1_cost_low == pt2_cost_low || pt1_cost_low == pt2_cost_high ||
      pt1_cost_high == pt2_cost_low || pt1_cost_high == pt2_cost_high) {
    return true;
  }
  int length =
      abs(pt1_cost_low - pt2_cost_high) > abs(pt2_cost_low - pt1_cost_high)
          ? abs(pt1_cost_low - pt2_cost_high)
          : abs(pt2_cost_low - pt1_cost_high);
  if (length < pt1_cost_high - pt1_cost_low + pt2_cost_high - pt2_cost_low) {
    return true;
  }

  return false;
}

void MYCBSROS::printAllResult() {
  std::cout << std::endl;
  std::cout << "Print All Result : " << std::endl;
  for (int i = 0; i < all_results_.size(); ++i) {
    for (int j = 0; j < all_results_[i].size(); ++j) {
      std::cout << "[" << all_results_[i][j].point_id << ", "
                << all_results_[i][j].cost << ", "
                << all_results_[i][j].wait_time << "], ";
    }
    std::cout << std::endl;
  }
}

bool MYCBSROS::assignTasks(const std::vector<int> &vehicles_init,
                           const std::vector<int> &start,
                           std::vector<int> &assign_result) {
  std::vector<std::pair<int, bool>> vehicle_states;
  vehicle_states.clear();
  assign_result.clear();
  task_assign_result_.clear();
  task_assign_result_.resize(start.size());
  assign_result.resize(start.size());
  for (int i = 0; i < vehicles_init.size(); i++) {
    vehicle_states.emplace_back(std::make_pair(vehicles_init[i], false));
  }

  //找到达每个任务点的最短路径
  std::vector<double> shortest_lengths;
  std::shared_ptr<std::vector<std::vector<float>>> graph =
      std::make_shared<std::vector<std::vector<float>>>(
          original_network_array_);
  map_parser::path_finder_algorithm::Dijkstra dij;
  for (int i = 0; i < start.size(); i++) {
    double min_dis = DBL_MAX;
    for (int j = 0; j < vehicles_init.size(); j++) {
      auto path =
          dij.findShortestPath(graph, vehicle_states[j].first, start[i]);
      double dis = calculatePathLength(path);
      if (dis < min_dis) {
        min_dis = dis;
      }
    }
    shortest_lengths.emplace_back(min_dis);
  }

  std::multimap<double, int> multi_maps;
  for (int i = 0; i < start.size(); i++) {
    multi_maps.insert(std::make_pair(shortest_lengths[i], i));
  }

  for (auto &iter : multi_maps) {
    double min_distance = DBL_MAX;
    int res = -1;
    for (int j = 0; j < vehicles_init.size(); j++) {
      if (vehicle_states[j].second == true)
        continue;
      std::shared_ptr<std::vector<std::vector<float>>> graph =
          std::make_shared<std::vector<std::vector<float>>>(
              original_network_array_);
      map_parser::path_finder_algorithm::Dijkstra dij;
      auto path = dij.findShortestPath(graph, vehicle_states[j].first,
                                       start[iter.second]);
      double dis = calculatePathLength(path);
      if (dis < min_distance) {
        res = j;
        min_distance = dis;
      }
    }
    // assign_result.emplace_back(vehicles_init[res]);
    assign_result[iter.second] = vehicles_init[res];
    //第iter.second任务分配给了第res辆车，未使用
    task_assign_result_[iter.second] = res;
    vehicle_states[res].second = true;
  }
  std::cout << "assign_result : ";
  for (int i = 0; i < assign_result.size(); i++) {
    std::cout << assign_result[i] << ", ";
  }
  std::cout << std::endl;

  return true;
}

double MYCBSROS::calculatePathLength(const std::vector<int> &path) {
  double length = 0.0;
  if (path.empty())
    return DBL_MAX;
  for (int i = 1; i < path.size(); i++) {
    length += original_network_array_[path[i - 1]][path[i]];
  }
  return length;
}

void MYCBSROS::firstOrderComeIn(const std::vector<int> &start,
                                const std::vector<int> &goal) {

  start_ids_ = start;
  goal_ids_ = goal;

  //收到订单后，对任务进行存储
  int size = start.size();
  task_list_.clear();
  std::vector<Task> tasks;
  for (int i = 0; i < size; i++) {
    Task task;
    task.task_id = task_list_.size();
    task.task_start_id = start[i];
    task.task_goal_id = goal[i];
    task.task_status = 1;
    task_list_.emplace_back(task);
    tasks.emplace_back(task);
  }

  mapf_msgs::GlobalPlan plan;

  std::vector<std::vector<int>> all_path_ids;

  double cost = 0;
  double time_tolerance = 5.0;
  double time = ros::Time::now().toSec();

  if (real_car_ == false) {
    makePlan(tasks, plan, cost, all_path_ids, time_tolerance);
  } else {
    makePlan(plan, cost, all_path_ids, time_tolerance, real_car_);
  }

  generateAllResult(all_path_ids, 0.2);

  mapf::MYCBSROS::Conflict conflict;

  while (firstConflictDetect(conflict) == true) {
    conflictSolve(conflict);
  }

  printAllResult();

  calculateAllPos();
}

void MYCBSROS::newOrderComeIn(const std::vector<int> &start,
                              const std::vector<int> &goal) {
  //第一步：保存现有的all_results_;
  auto original_all_results = all_results_;

  //新订单来临之后，存储原有机器人的新起点
  std::vector<ResultPoint> original_new_start_points;
  original_new_start_points.clear();
  original_new_start_points.resize(all_results_.size());

  std::vector<int> original_next_index;
  original_next_index.resize(all_results_.size());
  int point_count = map_nodes_.size();

  for (int i = 0; i < all_results_.size(); i++) {
    if (control_step_ - 1 >= all_pos_[i].size()) {
      ResultPoint pt;
      pt.cost = 0;
      pt.wait_time = 0;
      pt.point_id = -1;
      pt.x_y = std::make_pair(-1, -1);
      original_new_start_points[i] = pt;
      original_next_index[i] = -1;
      continue;
    }
    ResultPoint pt;
    pt.cost = 0;
    pt.wait_time = 0;
    pt.x_y = all_pos_[i][control_step_ - 1];
    pt.point_id = point_count++;
    original_new_start_points[i] = pt;
    for (int j = 0; j < all_results_[i].size() - 1; j++) {
      if (control_step_ - 1 > all_results_[i][j].cost) {
        original_next_index[i] = j + 1;
      }
    }
  }

  original_all_results.clear();
  for (int i = 0; i < all_results_.size(); i++) {
    if (original_next_index[i] == -1) {
      original_all_results.emplace_back();
      continue;
    }

    std::vector<ResultPoint> one_path;
    one_path.clear();
    one_path.emplace_back(original_new_start_points[i]);
    one_path.insert(one_path.end(),
                    all_results_[i].begin() + original_next_index[i],
                    all_results_[i].end());
    original_all_results.emplace_back(one_path);
  }

  //更新original_all_results里的cost!!!
  for (int i = 0; i < original_all_results.size(); i++) {
    for (int j = 1; j < original_all_results[i].size(); j++) {
      double length = std::hypot(original_all_results[i][j].x_y.first -
                                     original_all_results[i][j - 1].x_y.first,
                                 original_all_results[i][j].x_y.second -
                                     original_all_results[i][j - 1].x_y.second);
      int expand_step = std::ceil(length / 0.2);

      original_all_results[i][j].cost =
          original_all_results[i][j - 1].cost + expand_step;
    }
  }

  //第二步：将已有的路径截掉走过的路径，重新保存all_path_ids_

  //第三步：重新规划新传进来的任务

  //第四步：将原来的all_path_ids_存入新的all_path_ids_

  mapf_msgs::GlobalPlan plan;
  std::vector<std::vector<int>> all_path_ids;
  double cost = 0;
  double time_tolerance = 5.0;

  //新订单测试
  // std::vector<int> starts{55, 38};
  // std::vector<int> goals{40, 13};

  int size = start.size();
  std::vector<Task> tasks;
  for (int i = 0; i < size; i++) {
    Task task;
    task.task_id = task_list_.size();
    task.task_start_id = start[i];
    task.task_goal_id = goal[i];
    task.task_status = 1;
    task_list_.emplace_back(task);
    tasks.emplace_back(task);
  }

  makePlan(tasks, plan, cost, all_path_ids, time_tolerance);

  std::cout << "Plan result : " << std::endl;

  generateAllResult(all_path_ids, 0.2);

  //把两个部分合并
  all_results_.insert(all_results_.begin(), original_all_results.begin(),
                      original_all_results.end());

  generateAllRoutePathForShow();

  //第五步：进行冲突检测及冲突消解
  mapf::MYCBSROS::Conflict conflict;
  while (firstConflictDetect(conflict) == true) {

    conflictSolve(conflict);
  }

  printAllResult();

  //第六步：重新计算新的all_pos_
  calculateAllPos();

  control_step_ = 0;
}

void MYCBSROS::backToGarage(const int task_id) {

  //第一步：保存现有的all_results_;
  std::cout << "task id : " << task_id << std::endl;
  auto original_all_results = all_results_;

  std::vector<ResultPoint> original_new_start_points;
  original_new_start_points.clear();
  original_new_start_points.resize(all_results_.size());

  //存储原始未完成任务路径的下一个起点
  std::vector<int> original_next_index;
  original_next_index.resize(all_results_.size());
  int point_count = map_nodes_.size();

  for (int i = 0; i < all_results_.size(); i++) {
    //已完成任务的车辆，不需要重新标记，需要为它规划回车库的路径
    if (i == task_id)
      continue;
    //标记未完成任务的车辆的新位置
    if (control_step_ - 1 >= all_pos_[i].size()) {
      ResultPoint pt;
      pt.cost = 0;
      pt.wait_time = 0;
      pt.point_id = -1;
      pt.x_y = std::make_pair(-1, -1);
      original_new_start_points[i] = pt;
      original_next_index[i] = -1;
      continue;
    }
    ResultPoint pt;
    pt.cost = 0;
    pt.wait_time = 0;
    pt.x_y = all_pos_[i][control_step_ - 1];
    pt.point_id = point_count++;
    original_new_start_points[i] = pt;
    for (int j = 0; j < all_results_[i].size() - 1; j++) {
      if (control_step_ - 1 > all_results_[i][j].cost) {
        original_next_index[i] = j + 1;
      }
    }
  }

  original_all_results.clear();
  for (int i = 0; i < all_results_.size(); i++) {
    if (i == task_id) {
      original_all_results.emplace_back();
      continue;
    }

    if (original_next_index[i] == -1) {
      original_all_results.emplace_back();
      continue;
    }

    std::vector<ResultPoint> one_path;
    one_path.clear();
    one_path.emplace_back(original_new_start_points[i]);
    one_path.insert(one_path.end(),
                    all_results_[i].begin() + original_next_index[i],
                    all_results_[i].end());
    original_all_results.emplace_back(one_path);
  }

  //更新original_all_results里的cost!!!
  for (int i = 0; i < original_all_results.size(); i++) {
    for (int j = 1; j < original_all_results[i].size(); j++) {
      double length = std::hypot(original_all_results[i][j].x_y.first -
                                     original_all_results[i][j - 1].x_y.first,
                                 original_all_results[i][j].x_y.second -
                                     original_all_results[i][j - 1].x_y.second);
      int expand_step = std::ceil(length / 0.2);

      original_all_results[i][j].cost =
          original_all_results[i][j - 1].cost + expand_step;
    }
  }

  mapf_msgs::GlobalPlan plan;
  std::vector<std::vector<int>> all_path_ids;
  double cost = 0;
  double time_tolerance = 5.0;

  int car_id = task_list_[task_id].execution_car_id;
  int goal_id = garage_list_[vehicle_list_[car_id].garage_id].map_point_id;

  int start_id = task_list_[task_id].task_goal_id;

  std::cout << "check5" << std::endl;
  //选择最近的车库返回
  double min_dis = 1000000.0;
  int min_index = -1;
  std::vector<std::vector<int>> path_ids;
  for (int i = 0; i < garage_list_.size(); i++) {
    Task single_task;
    single_task.task_goal_id = garage_list_[i].map_point_id;
    single_task.task_start_id = start_id;
    all_path_ids.clear();
    makePlan(single_task, plan, cost, all_path_ids, time_tolerance);
    double length = 0.0;
    for (int i = 1; i < all_path_ids.front().size(); i++) {
      length += std::hypot(map_nodes_[i].x_pos - map_nodes_[i - 1].x_pos,
                           map_nodes_[i].y_pos - map_nodes_[i - 1].y_pos);
    }
    if (length < min_dis) {
      min_dis = length;
      min_index = i;
      path_ids.clear();
      path_ids = all_path_ids;
    }
  }

  std::cout << "min_index : " << min_index << std::endl;

  //更新task列表
  task_list_[task_id].task_start_id = start_id;
  task_list_[task_id].task_goal_id = garage_list_[min_index].map_point_id;
  task_list_[task_id].task_status = 3;

  //更新车辆状态
  vehicle_list_[car_id].car_status = 3;

  std::cout << "check5" << std::endl;
  //将车辆从原来的车库中移除
  for (int i = 0; i < garage_list_[vehicle_list_[car_id].garage_id]
                          .parking_vehicle_index.size();
       i++) {
    if (garage_list_[vehicle_list_[car_id].garage_id]
            .parking_vehicle_index[i] == car_id) {
      garage_list_[vehicle_list_[car_id].garage_id].parking_vehicle_index.erase(
          garage_list_[vehicle_list_[car_id].garage_id]
              .parking_vehicle_index.begin() +
          i);
      break;
    }
  }
  std::cout << "check5" << std::endl;
  //给车辆重新分配一个车库
  vehicle_list_[car_id].garage_id = min_index;

  std::cout << "Plan result" << std::endl;

  generateAllResult(path_ids, 0.2);

  std::cout << "all_result size : " << all_results_.size() << std::endl;
  std::cout << "all result front nums : " << all_results_.front().size()
            << std::endl;
  // std::cout << "all result back nums : " << all_results_.back().size()
  //           << std::endl;

  original_all_results[task_id] = all_results_.front();

  std::cout << "original_all_results[" << task_id
            << "] : " << original_all_results[task_id].size() << std::endl;

  all_results_.clear();

  all_results_ = original_all_results;

  generateAllRoutePathForShow();

  //第五步：进行冲突检测及冲突消解
  mapf::MYCBSROS::Conflict conflict;
  while (firstConflictDetect(conflict) == true) {

    conflictSolve(conflict);
  }

  printAllResult();

  //第六步：重新计算新的all_pos_
  calculateAllPos();

  control_step_ = 0;
}

int MYCBSROS::findNextId(const int task_id) {
  double cur_x = all_pos_[task_id][control_step_].first;
  double cur_y = all_pos_[task_id][control_step_].second;
  int min_index = 0;
  double dis = hypot(all_results_[task_id][0].x_y.first - cur_x,
                     all_results_[task_id][0].x_y.second - cur_y);
  for (int i = 1; i < all_results_[task_id].size(); i++) {
    double tmp_dis = hypot(all_results_[task_id][i].x_y.first - cur_x,
                           all_results_[task_id][i].x_y.second - cur_y);
    if (tmp_dis < dis) {
      dis = tmp_dis;
      min_index = i;
    }
  }
  if (min_index == 0) {
    return all_results_[task_id][1].point_id;
  } else {
    double len1 = hypot(all_results_[task_id][min_index].x_y.first -
                            all_results_[task_id][min_index - 1].x_y.first,
                        all_results_[task_id][min_index].x_y.second -
                            all_results_[task_id][min_index - 1].x_y.second);
    double len2 =
        hypot(all_results_[task_id][min_index - 1].x_y.first - cur_x,
              all_results_[task_id][min_index - 1].x_y.second - cur_y);
    if (len1 > len2)
      return all_results_[task_id][min_index].point_id;
    else
      return all_results_[task_id][(min_index + 1)].point_id;
  }
}

MYCBSROS::~MYCBSROS() {
  delete map_parser_;
  ROS_INFO("Exit MYCBS planner.");
}
} // namespace mapf

int main(int argc, char **argv) {

  ros::init(argc, argv, "my_cbs_test_node");
  ros::NodeHandle nh("~");
  mapf::MYCBSROS my_cbs_ros(&nh);

  //车辆的初始位置
  std::vector<int> vehicles_init; //{1, 2, 3, 4}

  if (my_cbs_ros.real_car_ == false) {
    vehicles_init = {11, 13, 20, 31};
  } else {
    vehicles_init = {0};
  }

  my_cbs_ros.vehicle_init_ids_ = vehicles_init;

  //初始化车库编号和位置
  my_cbs_ros.garage_list_.clear();
  my_cbs_ros.garage_list_.emplace_back();
  my_cbs_ros.garage_list_.back().garage_id = 0;
  my_cbs_ros.garage_list_.back().map_point_id = 59;

  my_cbs_ros.garage_list_.emplace_back();
  my_cbs_ros.garage_list_.back().garage_id = 1;
  my_cbs_ros.garage_list_.back().map_point_id = 60;

  //初始化车辆位置及初始停靠的车库
  my_cbs_ros.vehicle_list_.clear();
  for (int i = 0; i < 4; i++) {
    mapf::MYCBSROS::Vehicle vehicle;
    vehicle.car_id = i;
    vehicle.car_status = 1;
    vehicle.garage_id = i % 2;
    my_cbs_ros.garage_list_[i % 2].parking_vehicle_index.emplace_back(i);
    my_cbs_ros.vehicle_list_.emplace_back(vehicle);
  }

  ros::Rate loop(10); // 10hz的循环频率

  std::cout << "waiting order cmd..." << std::endl;
  while (my_cbs_ros.start_ids_.empty() && ros::ok()) {
    //显示地图
    my_cbs_ros.map_parser_->publishMapArray();
    my_cbs_ros.map_parser_->publishMapArrow();
    my_cbs_ros.publishOneStepPos(-1);
    ros::spinOnce();
    loop.sleep();
  }

  ros::Rate loop_rate(10);
  my_cbs_ros.control_step_ = 0;

  while (ros::ok()) {
    my_cbs_ros.map_parser_->publishMapArray();
    my_cbs_ros.map_parser_->publishMapArrow();

    my_cbs_ros.publishStartGoalPoint();

    // std::cout << all_path_ids.size() << std::endl;
    my_cbs_ros.updateAllRoutePathForShow();
    my_cbs_ros.map_parser_->publishAllRoutePath(
        my_cbs_ros.all_route_path_for_show_);

    my_cbs_ros.publishOneStepPos(my_cbs_ros.control_step_);
    my_cbs_ros.control_step_ = my_cbs_ros.control_step_ + 1;

    // if (path_found) {
    // map_parser.publishRouteNetwork(path_ids);
    // }
    ros::spinOnce();
    loop_rate.sleep(); // sleep 0.1s
  }

  ros::spin();

  return 0;
}