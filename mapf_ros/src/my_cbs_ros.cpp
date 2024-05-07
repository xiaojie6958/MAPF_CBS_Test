/*
 * @Descripttion:
 * @version:
 * @Author: CyberC3
 * @Date: 2024-04-14 22:57:43
 * @LastEditors: zhu-hu
 * @LastEditTime: 2024-05-04 22:31:30
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

  pub_start_goal_point_ =
      nh_->advertise<visualization_msgs::MarkerArray>("/start_gaol_points", 5);

  pub_start_goal_text_ =
      nh_->advertise<visualization_msgs::MarkerArray>("/start_goal_texts", 5);

  pub_one_step_pos_ =
      nh_->advertise<visualization_msgs::MarkerArray>("/one_step_pos", 5);
}

MYCBSROS::MYCBSROS(std::string name) : initialized_(false) { initialize(name); }

void MYCBSROS::initialize(std::string name) {
  if (!initialized_) {
    // ROS_INFO("New CBS planner.");

    initialized_ = true;
  }
}

bool MYCBSROS::makePlan(const std::vector<int> &start,
                        const std::vector<int> &goal,
                        mapf_msgs::GlobalPlan &plan, double &cost,
                        std::vector<std::vector<int>> &all_path_ids,
                        const double &time_tolerance) {
  // until tf can handle transforming things that are way in the past... we'll
  // require the goal to be in our global frame

  if (start.empty() || goal.empty()) {
    ROS_ERROR("Start and goal vectors are empty!");
    return false;
  }
  if (start.size() != goal.size()) {
    ROS_ERROR("Start and goal vectors are not the same length!");
    return false;
  }

  int agent_num = start.size();
  start_ids_ = start;
  goal_ids_ = goal;
  // mapf env
  std::vector<State> startStates;
  std::vector<Location> goals;

  // get agents start pose in world frame
  for (int i = 0; i < agent_num; ++i) {
    // transform to map form
    startStates.emplace_back(State(0, 0, 0, start[i]));

    goals.emplace_back(Location(0, 0, goal[i]));

  } // end for

  // mapf search

  std::vector<PlanResult<State, Action, int>> solution;

  std::cout << "map_size : " << map_nodes_.size() << ", "
            << node_name_to_id_.size() << ", " << node_id_to_name_.size()
            << original_network_array_.size()
            << original_network_array_[0].size() << std::endl;
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
    std::cout << "check" << std::endl;
    generatePlan(solution, goal, plan, cost);

    all_path_ids.clear();
    for (int i = 0; i < solution.size(); ++i) {
      std::vector<int> path_id;
      path_id.clear();
      for (int j = 0; j < solution[i].states.size(); j++) {
        path_id.emplace_back(solution[i].states[j].first.id);
      }
      all_path_ids.emplace_back(path_id);
    }

    // all_path_ids_ = std::move(all_path_ids);
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

void MYCBSROS::calculateAllPos() {
  all_pos_.clear();
  std::vector<std::pair<double, double>> one_path;
  double step_length = 0.2;
  for (int i = 0; i < all_path_ids_.size(); i++) {
    one_path.clear();
    for (int j = 1; j < all_path_ids_[i].size(); j++) {
      one_path.emplace_back(
          std::make_pair(map_nodes_[all_path_ids_[i][j - 1]].x_pos,
                         map_nodes_[all_path_ids_[i][j - 1]].y_pos));
      int wait_time = all_results_[i][j - 1].wait_time;
      while (wait_time > 0) {
        one_path.emplace_back(
            std::make_pair(map_nodes_[all_path_ids_[i][j - 1]].x_pos,
                           map_nodes_[all_path_ids_[i][j - 1]].y_pos));
        wait_time--;
      }
      double length = std::hypot(map_nodes_[all_path_ids_[i][j]].x_pos -
                                     map_nodes_[all_path_ids_[i][j - 1]].x_pos,
                                 map_nodes_[all_path_ids_[i][j]].y_pos -
                                     map_nodes_[all_path_ids_[i][j - 1]].y_pos);
      int expand_step = std::ceil(length / step_length);
      std::pair<double, double> unit{
          (map_nodes_[all_path_ids_[i][j]].x_pos -
           map_nodes_[all_path_ids_[i][j - 1]].x_pos) /
              length,
          (map_nodes_[all_path_ids_[i][j]].y_pos -
           map_nodes_[all_path_ids_[i][j - 1]].y_pos) /
              length};
      for (int k = 1; k < expand_step; k++) {
        one_path.emplace_back(
            std::make_pair(map_nodes_[all_path_ids_[i][j - 1]].x_pos +
                               k * unit.first * step_length,
                           map_nodes_[all_path_ids_[i][j - 1]].y_pos +
                               k * unit.second * step_length));
      }
    }
    one_path.emplace_back(
        std::make_pair(map_nodes_[all_path_ids_[i].back()].x_pos,
                       map_nodes_[all_path_ids_[i].back()].y_pos));
    std::cout << "one path[" << i << "] nums : " << one_path.size()
              << std::endl;
    all_pos_.emplace_back(one_path);
  }
}

void MYCBSROS::publishOneStepPos(const int step) {
  int count = 0;
  one_step_pos_.markers.clear();
  visualization_msgs::Marker point_marker;
  point_marker.header.frame_id = "world";
  point_marker.header.stamp = ros::Time::now();
  point_marker.ns = "all_points";
  point_marker.action = visualization_msgs::Marker::ADD;
  point_marker.lifetime = ros::Duration(0);
  point_marker.type = visualization_msgs::Marker::CYLINDER;
  point_marker.scale.x = 1.5;
  point_marker.scale.y = 1.5;
  point_marker.scale.z = 1.5;
  for (int i = 0; i < all_pos_.size(); i++) {
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
    if (step >= all_pos_[i].size()) {
      point_marker.pose.position.x = all_pos_[i].back().first;
      point_marker.pose.position.y = all_pos_[i].back().second;
    } else {
      point_marker.pose.position.x = all_pos_[i][step].first;
      point_marker.pose.position.y = all_pos_[i][step].second;
    }

    count++;

    one_step_pos_.markers.emplace_back(point_marker);
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
  point_marker.lifetime = ros::Duration(0);
  point_marker.type = visualization_msgs::Marker::CYLINDER;
  point_marker.scale.x = 2.0;
  point_marker.scale.y = 2.0;
  point_marker.scale.z = 2.0;
  for (int i = 0; i < start_ids_.size(); ++i) {
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
    point_marker.pose.position.x = map_nodes_[start_ids_[i]].x_pos;
    point_marker.pose.position.y = map_nodes_[start_ids_[i]].y_pos;
    count++;

    start_goal_points_.markers.emplace_back(point_marker);
  }

  int color_count = 0;
  point_marker.type = visualization_msgs::Marker::CUBE;
  for (int i = 0; i < goal_ids_.size(); ++i) {
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
    point_marker.pose.position.x = map_nodes_[goal_ids_[i]].x_pos;
    point_marker.pose.position.y = map_nodes_[goal_ids_[i]].y_pos;
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
  text_marker.lifetime = ros::Duration(0);
  // Scale
  text_marker.scale.x = 2.0;
  text_marker.scale.y = 2.0;
  text_marker.scale.z = 2.0;
  // Color
  text_marker.color.r = 0.0;
  text_marker.color.g = 0.0;
  text_marker.color.b = 0.0;
  text_marker.color.a = 1.0;
  for (int i = 0; i < start_ids_.size(); i++) {
    text_marker.id = id_count++;
    text_marker.pose.position.x = map_nodes_[start_ids_[i]].x_pos;
    text_marker.pose.position.y = map_nodes_[start_ids_[i]].y_pos;
    text_marker.pose.position.z = 0.0;
    text_marker.text = "S" + std::to_string(i);
    start_goal_texts_.markers.emplace_back(text_marker);
  }

  for (int i = 0; i < goal_ids_.size(); i++) {
    text_marker.id = id_count++;
    text_marker.pose.position.x = map_nodes_[goal_ids_[i]].x_pos;
    text_marker.pose.position.y = map_nodes_[goal_ids_[i]].y_pos;
    text_marker.pose.position.z = 0.0;
    text_marker.text = "G" + std::to_string(i);
    start_goal_texts_.markers.emplace_back(text_marker);
  }

  pub_start_goal_text_.publish(start_goal_texts_);
}

void MYCBSROS::generateAllResult(const double step_length) {
  all_results_.clear();
  for (int i = 0; i < all_path_ids_.size(); ++i) {
    std::vector<ResultPoint> one_path;
    one_path.clear();
    one_path.resize(all_path_ids_[i].size());
    ResultPoint pt;
    pt.point_id = all_path_ids_[i][0];
    pt.cost = 0;
    pt.wait_time = 0;
    one_path[0] = pt;
    for (int j = 1; j < all_path_ids_[i].size(); ++j) {
      pt.point_id = all_path_ids_[i][j];
      double length = std::hypot(map_nodes_[all_path_ids_[i][j]].x_pos -
                                     map_nodes_[all_path_ids_[i][j - 1]].x_pos,
                                 map_nodes_[all_path_ids_[i][j]].y_pos -
                                     map_nodes_[all_path_ids_[i][j - 1]].y_pos);
      int expand_step = std::ceil(length / step_length);
      pt.cost = one_path[j - 1].cost + expand_step;
      one_path[j] = pt;
    }
    all_results_.emplace_back(one_path);
  }

  for (int i = 0; i < all_results_.size(); ++i) {
    for (int j = 0; j < all_results_[i].size(); ++j) {
      std::cout << "[" << all_results_[i][j].point_id << ", "
                << all_results_[i][j].cost << ", "
                << all_results_[i][j].wait_time << "], ";
    }
    std::cout << std::endl;
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
  for (int i = 0; i < all_path_ids_.size(); ++i) {
    for (int j = i + 1; j < all_path_ids_.size(); ++j) {
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

MYCBSROS::~MYCBSROS() {
  delete map_parser_;
  ROS_INFO("Exit MYCBS planner.");
}
} // namespace mapf

//   ros::init(argc, argv, "map_parser");

//   ros::NodeHandle nh("~");

//   map_parser::MapParser map_parser(&nh);

//   map_parser::path_finder_algorithm::Dijkstra dijkstra_planner;

//   ros::Rate loop_rate(10);

//   map_parser.parse();

//   int source_id = 2;

//   int target_id = 6;

//   bool path_found = false;

//   std::vector<int> path_ids;

//   std::cout << "source_id : " << source_id << std::endl;
//   std::cout << "target_id : " << target_id << std::endl;

//   try {
//     double start_time = ros::Time::now().toSec();
//     path_ids =
//     dijkstra_planner.findShortestPath(map_parser.getGraphOfVertex(),
//                                                  source_id, target_id);
//     path_found = true;

//     ROS_INFO("OSM planner: Time of planning : %f  ms",
//              1000.0 * (ros::Time::now().toSec() - start_time));

//   } catch (map_parser::path_finder_algorithm::PathFinderException &e) {
//     path_found = false;
//     if (e.getErrId() ==
//         map_parser::path_finder_algorithm::PathFinderException::NO_PATH_FOUND)
//         {
//       ROS_ERROR("OSM planner: Make plan failed...");
//     } else
//       ROS_ERROR("OSM planner: Undefined error");
//   }

//   if (path_ids.size() != 0) {
//     std::cout << "planning results from [" << source_id << "] to [" <<
//     target_id
//               << "]" << std::endl;
//     for (int i = 0; i < path_ids.size(); i++) {
//       std::cout << path_ids[i] << ", ";
//     }
//     std::cout << std::endl;

//     std::cout << "name results : ";
//     for (int i = 0; i < path_ids.size(); i++) {
//       std::cout << (map_parser.node_id_to_name())[path_ids[i]] << ", ";
//     }
//     std::cout << std::endl;
//   }

//   while (ros::ok()) {
//     map_parser.publishMapArray();
//     map_parser.publishMapArrow();

//     if (path_found) {
//       map_parser.publishRouteNetwork(path_ids);
//     }
//     ros::spinOnce();
//     loop_rate.sleep(); // sleep 0.1s
//   }

//   return 0;

int main(int argc, char **argv) {

  ros::init(argc, argv, "my_cbs_test_node");
  ros::NodeHandle nh("~");
  mapf::MYCBSROS my_cbs_ros(&nh);

  std::vector<int> start{0, 5, 2, 16, 55, 38};

  std::vector<int> goal{32, 10, 9, 29, 40, 13};

  mapf_msgs::GlobalPlan plan;
  std::vector<std::vector<int>> all_path_ids;
  double cost = 0;
  double time_tolerance = 5.0;

  double time = ros::Time::now().toSec();
  my_cbs_ros.makePlan(start, goal, plan, cost, all_path_ids, time_tolerance);

  std::cout << "Plan result : " << std::endl;

  for (int i = 0; i < plan.global_plan.size(); ++i) {
    std::cout << "plan[" << i << "] : ";
    for (int j = 0; j < plan.global_plan[i].plan.poses.size(); ++j) {
      std::cout << "(" << plan.global_plan[i].plan.poses[j].pose.position.x
                << ", " << plan.global_plan[i].plan.poses[j].pose.position.y
                << "), ";
    }
    std::cout << std::endl;
  }

  my_cbs_ros.generateAllResult(0.2);

  mapf::MYCBSROS::Conflict conflict;
  while (my_cbs_ros.firstConflictDetect(conflict) == true) {

    my_cbs_ros.conflictSolve(conflict);
  }

  std::cout << "Plan time : " << ros::Time::now().toSec() - time << std::endl;

  my_cbs_ros.printAllResult();

  my_cbs_ros.calculateAllPos();

  ros::Rate loop_rate(10);
  int step = 0;

  while (ros::ok()) {
    my_cbs_ros.map_parser_->publishMapArray();
    my_cbs_ros.map_parser_->publishMapArrow();

    my_cbs_ros.publishStartGoalPoint();

    // std::cout << all_path_ids.size() << std::endl;

    my_cbs_ros.map_parser_->publishAllRoutePath(all_path_ids);

    my_cbs_ros.publishOneStepPos(step);
    step++;

    // if (path_found) {
    //   map_parser.publishRouteNetwork(path_ids);
    // }
    ros::spinOnce();
    loop_rate.sleep(); // sleep 0.1s
  }

  ros::spin();

  return 0;
}