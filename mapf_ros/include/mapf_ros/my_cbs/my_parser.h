/*
 * @Descripttion:
 * @version:
 * @Author: CyberC3
 * @Date: 2024-05-19 22:52:46
 * @LastEditors: zhu-hu
 * @LastEditTime: 2024-05-19 23:56:28
 */
#pragma

#include <ros/ros.h>
#include <tinyxml.h>
#include <unordered_map>
#include <vector>

namespace my_map_parser {
class MyMapParser {
public:
  typedef struct point_pose {
    double x;
    double y;
  } POINT_POSE;

  typedef struct map_node {
    int id;
    double yaw;
    int type;
    std::string color;
    int scale;
    POINT_POSE center_pose;
    double height;
  } MAP_NODE;

  typedef struct map_path {
    int id;
    int type;
    std::string color;
    int width;
    double distance;
    int direction;
    int start_point_id;
    int end_point_id;
  } MAP_PATH;
  MyMapParser(ros::NodeHandle *nh);

  MyMapParser();

  void parse(bool onlyFirstElement = false);

  void createNetwork();

private:
  ros::NodeHandle *nh_;
  std::vector<MAP_NODE> map_nodes_;

  std::vector<MAP_PATH> map_paths_;

  std::vector<std::vector<float>> networkArray_;

  std::string xml_file_name_;

  void createPaths(TiXmlHandle *hRootWay, bool onlyFirstElement = false);

  void createNodes(TiXmlHandle *hRootNode, bool onlyFirstElement = false);
};
} // namespace my_map_parser