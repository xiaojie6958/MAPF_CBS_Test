/*
 * @Descripttion:
 * @version:
 * @Author: CyberC3
 * @Date: 2024-05-19 22:53:00
 * @LastEditors: zhu-hu
 * @LastEditTime: 2024-05-23 21:19:16
 */
#include "my_parser.h"
#include "path_finder_algorithm/dijkstra.h"

namespace my_map_parser {

MyMapParser::MyMapParser(ros::NodeHandle *nh) : nh_(nh) {
  nh_->param<std::string>("my_map_path", xml_file_name_, " ");
}

MyMapParser::MyMapParser() {}

void MyMapParser::parse(bool onlyFirstElement) {
  TiXmlDocument doc(xml_file_name_);
  TiXmlNode *xml_map;
  TiXmlNode *node;
  TiXmlNode *path;

  TiXmlHandle hRootNode(0);
  TiXmlHandle hRootPath(0);

  bool loadOkay = doc.LoadFile();

  if (loadOkay) {
    ROS_INFO("My Map parser: loaded map: %s", xml_file_name_.c_str());
  } else {
    ROS_ERROR("My Map parser: Failed to load file %s", xml_file_name_.c_str());
    throw std::runtime_error("Failed to load xml");
  }

  xml_map = doc.FirstChildElement();

  node = xml_map->FirstChild("point");
  path = xml_map->FirstChild("path");

  TiXmlElement *nodeElement = node->ToElement();

  TiXmlElement *pathElement = path->ToElement();

  hRootNode = TiXmlHandle(nodeElement);
  hRootPath = TiXmlHandle(pathElement);

  createPaths(&hRootPath, onlyFirstElement);
  createNodes(&hRootNode, onlyFirstElement);

  createNetwork();

  if (onlyFirstElement)
    return;

  std::cout << "MyMapParser Init Finshed!!!" << std::endl;
}

void MyMapParser::createNodes(TiXmlHandle *hRootNode, bool onlyFirstElement) {

  TiXmlElement *nodeElement = hRootNode->Element();

  int node_id = 0;
  for (nodeElement; nodeElement;
       nodeElement = nodeElement->NextSiblingElement("point")) {

    MAP_NODE tmp_node;

    tmp_node.id = node_id++;
    nodeElement->Attribute("xPosition", &(tmp_node.x_pos));
    nodeElement->Attribute("yPosition", &(tmp_node.y_pos));
    nodeElement->Attribute("zPosition", &(tmp_node.z_pos));
    //将单位从mm换算成m
    tmp_node.x_pos = tmp_node.x_pos * 0.001;
    tmp_node.y_pos = tmp_node.y_pos * 0.001;
    tmp_node.z_pos = tmp_node.z_pos * 0.001;
    nodeElement->Attribute("vehicleOrientationAngle", &(tmp_node.yaw));
    tmp_node.type = nodeElement->Attribute("type");

    TiXmlElement *outgoing_path = nullptr;
    outgoing_path = nodeElement->FirstChildElement("outgoingPath");
    while (outgoing_path != nullptr) {
      OutgoingPath path;
      path.name = outgoing_path->Attribute("name");
      tmp_node.outgoing_path.emplace_back(path);

      outgoing_path = outgoing_path->NextSiblingElement("outgoingPath");
    }

    TiXmlElement *property = nullptr;
    property = nodeElement->FirstChildElement("property");
    if (property != nullptr) {
      tmp_node.point_property.name = property->Attribute("name");
      property->Attribute("value", &(tmp_node.point_property.value));
    }

    TiXmlElement *point_layout = nullptr;
    point_layout = nodeElement->FirstChildElement("pointLayout");
    if (point_layout != nullptr) {
      point_layout->Attribute("xPosition", &(tmp_node.point_layout.x_pos));
      point_layout->Attribute("yPosition", &(tmp_node.point_layout.y_pos));
      point_layout->Attribute("xLabelOffset",
                              &(tmp_node.point_layout.x_label_offset));
      point_layout->Attribute("yLabelOffset",
                              &(tmp_node.point_layout.y_label_offset));
      //将单位从mm换算成m
      tmp_node.point_layout.x_pos = tmp_node.point_layout.x_pos * 0.001;
      tmp_node.point_layout.y_pos = tmp_node.point_layout.y_pos * 0.001;
      tmp_node.point_layout.x_label_offset =
          tmp_node.point_layout.x_label_offset * 0.001;
      tmp_node.point_layout.y_label_offset =
          tmp_node.point_layout.y_label_offset * 0.001;
      point_layout->Attribute("layerId", &(tmp_node.point_layout.layer_id));
    }

    map_nodes_.emplace_back(tmp_node);

    if (onlyFirstElement)
      return;
  }
}

void MyMapParser::createPaths(TiXmlHandle *hRootWay, bool onlyFirstElement) {
  map_paths_.clear();

  TiXmlElement *pathElement = hRootWay->Element();

  int path_id = 0;
  for (pathElement; pathElement;
       pathElement = pathElement->NextSiblingElement("path")) {

    MAP_PATH tmp_path;

    tmp_path.path_id = path_id++;
    tmp_path.source_node_name = pathElement->Attribute("sourcePoint");
    tmp_path.destination_node_name = pathElement->Attribute("destinationPoint");
    pathElement->Attribute("length", &(tmp_path.length));
    pathElement->Attribute("maxVelocity", &(tmp_path.max_vel));
    pathElement->Attribute("maxReverseVelocity", &(tmp_path.max_reverse_vel));
    tmp_path.locked = pathElement->Attribute("locked");
    //将单位转换为m或者m/s
    tmp_path.length = tmp_path.length * 0.001;
    tmp_path.max_vel = tmp_path.max_vel * 0.001;
    tmp_path.max_reverse_vel = tmp_path.max_reverse_vel * 0.001;

    TiXmlElement *pathLayoutElement = nullptr;
    pathLayoutElement = pathElement->FirstChildElement("pathLayout");

    if (pathLayoutElement != nullptr) {
      tmp_path.path_layout.connection_type =
          pathLayoutElement->Attribute("connectionType");
      pathLayoutElement->Attribute("layerId", &(tmp_path.path_layout.layer_id));

      if (tmp_path.path_layout.connection_type == "BEZIER") {
        TiXmlElement *controlPointElement =
            pathLayoutElement->FirstChildElement("controlPoint");
        while (controlPointElement != nullptr) {
          ControlPoint control_pt;
          controlPointElement->Attribute("x", &(control_pt.x));
          controlPointElement->Attribute("y", &(control_pt.y));
          //将单位转换为m
          control_pt.x = control_pt.x * 0.001;
          control_pt.y = control_pt.y * 0.001;
          tmp_path.path_layout.control_points.emplace_back(control_pt);
          controlPointElement =
              controlPointElement->NextSiblingElement("controlPoint");
        }
      }
    }

    map_paths_.emplace_back(tmp_path);

    if (onlyFirstElement)
      return;
  }
}

void MyMapParser::createNetwork() {
  networkArray_.clear();
  networkArray_.resize(map_nodes_.size(),
                       std::vector<float>(map_nodes_.size(), 0.0));

  double distance = 0.0;

  for (int i = 0; i < map_paths_.size(); i++) {

    std::string src_name = map_paths_[i].source_node_name;
    std::string dst_name = map_paths_[i].destination_node_name;

    int src_id = node_name_to_id_[src_name];

    int dst_id = node_name_to_id_[dst_name];

    distance = map_paths_[i].length;

    networkArray_[src_id][dst_id] = (float)distance;
    // networkArray_[dst_id][src_id] = (float)distance;
  }
}
} // namespace my_map_parser

#include <climits>
#include <iostream>
#include <limits>
#include <queue>
#include <unordered_map>
#include <vector>

// 地图节点的表示
struct Node {
  std::string name;                               // 节点名称
  std::unordered_map<std::string, int> neighbors; // 相邻节点及距离
};

// AGV的表示
struct AGV {
  std::string currentPosition;
  bool isAssigned;
};

// 任务（路径起点）的表示
struct Task {
  std::string startPosition;
};

// 计算两个节点之间的最短距离
int shortestDistance(const std::unordered_map<std::string, Node> &map,
                     const std::string &start, const std::string &end) {
  // 这里可以使用Dijkstra算法或其他图搜索算法来计算最短路径
  // 为简化起见，我们假设地图是简单的，并且直接返回相邻节点的距离
  if (map.at(start).neighbors.count(end)) {
    return map.at(start).neighbors.at(end);
  }
  return INT_MAX; // 如果不可达，则返回一个大数
}

// 任务分配算法
std::unordered_map<std::string, std::string>
assignTasks(const std::unordered_map<std::string, Node> &map,
            const std::vector<AGV> &agvs, const std::vector<Task> &tasks) {
  std::unordered_map<std::string, std::string> assignments; // 分配结果
  std::priority_queue<std::pair<int, std::string>,
                      std::vector<std::pair<int, std::string>>,
                      std::greater<std::pair<int, std::string>>>
      taskQueue;

  // 将任务按距离起点的估计距离排序（这里简化为直接入队）
  for (const auto &task : tasks) {
    taskQueue.push({0, task.startPosition});
  }

  // 初始化AGV的分配状态
  std::unordered_map<std::string, bool> agvAssigned;
  for (const auto &agv : agvs) {
    agvAssigned[agv.currentPosition] = false;
  }

  while (!taskQueue.empty()) {
    auto currentTask = taskQueue.top();
    taskQueue.pop();

    std::string taskPosition = currentTask.second;
    int minDistance = INT_MAX;
    std::string assignedAGV;

    // 寻找最近的未分配任务的AGV
    for (const auto &agv : agvs) {
      if (!agvAssigned[agv.currentPosition]) {
        int distance = shortestDistance(map, agv.currentPosition, taskPosition);
        if (distance < minDistance) {
          minDistance = distance;
          assignedAGV = agv.currentPosition;
        }
      }
    }

    if (!assignedAGV.empty()) {
      assignments[assignedAGV] = taskPosition;
      agvAssigned[assignedAGV] = true;
    }
  }

  return assignments;
}

int main() {
  // 构建地图、AGV和任务的示例数据
  std::unordered_map<std::string, Node> map = {
      {"A", {"A", {{"B", 10}, {"C", 15}}}},
      {"B", {"B", {{"A", 10}, {"D", 20}}}},
      {"C", {"C", {{"A", 15}, {"D", 12}}}},
      {"D", {"D", {{"B", 20}, {"C", 12}}}}};

  std::vector<AGV> agvs = {{"A", false}, {"B", false}};
  std::vector<Task> tasks = {{"C"}, {"D"}};

  // 执行任务分配
  auto assignments = assignTasks(map, agvs, tasks);

  // 输出分配结果
  for (const auto &assignment : assignments) {
    std::cout << "AGV at " << assignment.first << " is assigned to task at "
              << assignment.second << std::endl;
  }

  return 0;
}