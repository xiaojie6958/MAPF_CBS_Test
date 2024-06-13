/*
 * @Descripttion:
 * @version:
 * @Author: CyberC3
 * @Date: 2024-04-06 22:59:03
 * @LastEditors: zhu-hu
 * @LastEditTime: 2024-06-12 23:01:03
 */
#pragma once

#include <ros/ros.h>

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

// #include "mapf_ros/mapf_ros.hpp"
#include "my_cbs.hpp"
#include "my_cbs_env.hpp"

#include "parser.h"

#include "path_finder_algorithm/dijkstra.h"

//用于json的序列化和反序列化
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

namespace mapf {

class MYCBSROS {
public:
  struct ResultPoint {
    int point_id;
    int cost;
    int wait_time;
    std::pair<double, double> x_y;

    ResultPoint() {
      cost = 0;
      wait_time = 0;
    }
  };
  struct Conflict {
    std::pair<int, int> conflict_id;
    std::pair<int, int> point_ids;
  };
  MYCBSROS(ros::NodeHandle *nh);
  MYCBSROS(std::string name);

  void initialize(std::string name);

  bool makePlan(mapf_msgs::GlobalPlan &plan, double &cost,
                std::vector<std::vector<int>> &all_path_ids,
                const double &time_tolerance);

  bool makePlan(mapf_msgs::GlobalPlan &plan, double &cost,
                std::vector<std::vector<int>> &all_path_ids,
                const double &time_tolerance, bool real_car);

  void generatePlan(const std::vector<PlanResult<State, Action, int>> &solution,
                    const std::vector<int> &goal, mapf_msgs::GlobalPlan &plan,
                    double &cost);
  ~MYCBSROS();

  int control_step_ = 0;

  //所有车辆起始位置的id
  std::vector<int> vehicle_init_ids_;
  //所有任务起点位置的id
  std::vector<int> start_ids_;
  //所有任务终点位置的id
  std::vector<int> goal_ids_;

protected:
  bool initialized_;

  ros::NodeHandle *nh_;

  //存储规划后的所有路径点，两个阶段（车辆当前位置->任务起点位置->任务终点位置）的点都包含
  std::vector<std::vector<int>> all_path_ids_;

  //存储规划后的结果，两个阶段（车辆当前位置->任务起点位置->任务终点位置）的点都包含
  std::vector<std::vector<ResultPoint>> all_results_;

  //存储每一个机器人运动过程中的路径点，两个阶段（车辆当前位置->任务起点位置->任务终点位置）的点都包含
  std::vector<std::vector<std::pair<double, double>>> all_pos_;

  //存储任务分配的结果
  std::vector<int> task_assign_result_;

  //存储每一个任务中，两个阶段切换的点的下标索引
  std::vector<int> all_pos_switch_index_;

  std::vector<int> final_switch_index_;

  ros::Publisher pub_start_goal_point_;

  ros::Publisher pub_start_goal_text_;

  ros::Publisher pub_one_step_pos_;

  ros::Subscriber sub_new_order_;

  ros::Subscriber sub_order_from_web_;

  visualization_msgs::MarkerArray start_goal_points_;
  visualization_msgs::MarkerArray start_goal_texts_;
  visualization_msgs::MarkerArray one_step_pos_;

  void generateAllRoutePathForShow();

public:
  void calculateAllPos();
  void publishStartGoalPoint();
  void publishOneStepPos(const int step);
  void generateAllResult(const double step_length);

  void updateAllRoutePathForShow();

  std::vector<std::vector<std::pair<double, double>>> all_route_path_for_show_;

  //解决conflict中包含冲突
  bool conflictSolve(Conflict conflict);

  //检测path_i和path_j中的第一个冲突，存到conflict里
  bool conflictDetect(int path_i, int path_j, Conflict &conflict);

  //检测所有路径中的第一个冲突，存到conflict里
  bool firstConflictDetect(Conflict &conflict);

  //判断pt1和pt2是否有冲突
  bool twoResultPointConflict(const ResultPoint pt1, const ResultPoint pt2);

  void printAllResult();

  bool assignTasks(const std::vector<int> &vehicles_init,
                   const std::vector<int> &start,
                   std::vector<int> &assign_result);
  double calculatePathLength(const std::vector<int> &path);

  //增加新订单后的callback函数
  void newOrderCallback(const std_msgs::BoolConstPtr &msg_in);

  void setOrderCallback(const std_msgs::StringConstPtr &msg_in);
  map_parser::MapParser *map_parser_ = nullptr;

  //地图中所有的node节点
  std::vector<MapParser::MAP_NODE> map_nodes_;
  // node节点由名字到id的转换map
  std::unordered_map<std::string, int> node_name_to_id_;
  // node节点由id到名字的转换map
  std::unordered_map<int, std::string> node_id_to_name_;
  //地图的拓扑结构
  std::vector<std::vector<float>> original_network_array_;

  bool real_car_ = false;
};
} // namespace mapf