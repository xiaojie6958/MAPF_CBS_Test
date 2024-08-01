/*
 * @Descripttion:
 * @version:
 * @Author: CyberC3
 * @Date: 2024-03-17 11:26:30
 * @LastEditors: zhu-hu
 * @LastEditTime: 2024-08-01 22:52:15
 */
#pragma once

#include <ros/ros.h>
#include <tinyxml.h>
#include <unordered_map>
#include <vector>

#include <tf/transform_datatypes.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <nav_msgs/Path.h>

namespace map_parser {
class MapParser {
public:
  typedef struct {
    std::string name;
  } OutgoingPath;

  typedef struct {
    double x_pos;
    double y_pos;
    double x_label_offset;
    double y_label_offset;
    int layer_id;
  } PointLayout;

  typedef struct {
    std::string name;
    int value;
  } PointProperty;

  typedef struct map_node {
    std::string name;
    int node_id;
    double x_pos;
    double y_pos;
    double z_pos;
    double yaw;
    std::string type;
    std::vector<OutgoingPath> outgoing_path;
    PointProperty point_property;
    PointLayout point_layout;
  } MAP_NODE;

  typedef struct control_point {
    double x;
    double y;
  } ControlPoint;

  typedef struct {
    std::string connection_type;
    int layer_id;
    std::vector<ControlPoint> control_points;
  } PathLayout;

  typedef struct map_path {
    int path_id;
    std::string source_node_name;
    std::string destination_node_name;
    double length;
    double max_vel;
    double max_reverse_vel;
    std::string locked;
    PathLayout path_layout;
  } MAP_PATH;

  MapParser(ros::NodeHandle *nh);

  MapParser();

  // start parsing
  void parse(bool onlyFirstElement = false);

  void set_new_map(std::string xml_name);

  void set_type_of_path(std::vector<std::string> types);

  void createNetwork();

  std::shared_ptr<std::vector<std::vector<float>>> getGraphOfVertex();

  inline const std::vector<std::vector<float>> &road_network() const {
    return networkArray_;
  }

  inline const std::vector<MAP_NODE> &map_nodes() const { return map_nodes_; }

  inline std::unordered_map<int, std::string> &node_id_to_name() {
    return node_id_to_name_;
  }

  inline std::unordered_map<std::string, int> &node_name_to_id() {
    return node_name_to_id_;
  }

  void publishRouteNetwork();

  void publishMapArrow();

  void publishMapPoint();

  void publishMapArray();

  void drawCenterLine(const MapParser::MAP_PATH &map_path, int &count_id);

  void publishRouteNetwork(const std::vector<int> &route_paths_id);

  void drawOneRoutePath(const std::vector<int> &path_id, int &count_id);

  void drawOneRoutePath(const std::vector<std::pair<double, double>> &one_path,
                        int &count_id);

  void publishAllRoutePath(
      const std::vector<std::vector<std::pair<double, double>>> &route_paths);

  void publishAllRoutePath(const std::vector<std::vector<int>> &all_path_ids);

private:
  ros::NodeHandle *nh_;
  std::vector<MAP_NODE> map_nodes_;
  std::vector<MAP_PATH> map_paths_;

  std::unordered_map<std::string, int> node_name_to_id_;

  std::unordered_map<int, std::string> node_id_to_name_;

  std::vector<std::vector<float>> networkArray_;

  std::string xml_file_name_;

  const static int CURRENT_POSITION_MARKER = 0;
  const static int TARGET_POSITION_MARKER = 1;

  // visualization msgs
  visualization_msgs::Marker position_marker, target_marker;

  visualization_msgs::MarkerArray map_marker_array;

  visualization_msgs::MarkerArray map_point_markers;

  visualization_msgs::MarkerArray point_texts;

  visualization_msgs::MarkerArray all_route_paths_markers;

  //暂无用
  std::vector<std::string> type_of_path_;

  void createPaths(TiXmlHandle *hRootWay, bool onlyFirstElement = false);

  void createNodes(TiXmlHandle *hRootNode, bool onlyFirstElement = false);

  // publishers
  ros::Publisher position_marker_pub;
  ros::Publisher target_marker_pub;
  ros::Publisher path_pub;
  ros::Publisher map_pub;
  ros::Publisher map_point_pub;
  ros::Publisher text_pub;

  ros::Publisher all_route_paths_pub;

  // publishing functions
  void publishPoint(int pointID, int marker_type, double radius,
                    geometry_msgs::Quaternion orientation =
                        tf::createQuaternionMsgFromYaw(0));
  void publishPoint(geometry_msgs::Point point, int marker_type, double radius,
                    geometry_msgs::Quaternion orientation =
                        tf::createQuaternionMsgFromYaw(0));
  void publishPoint(const MAP_NODE &node, int marker_type, double radius,
                    geometry_msgs::Quaternion orientation =
                        tf::createQuaternionMsgFromYaw(0));
  void publishPoint(double latitude, double longitude, int marker_type,
                    double radius,
                    geometry_msgs::Quaternion orientation =
                        tf::createQuaternionMsgFromYaw(0));

  void publishPointText();
};
} // namespace map_parser