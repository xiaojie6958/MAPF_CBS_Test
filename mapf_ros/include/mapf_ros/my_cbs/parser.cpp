/*
 * @Descripttion:
 * @version:
 * @Author: CyberC3
 * @Date: 2024-03-17 11:26:22
 * @LastEditors: zhu-hu
 * @LastEditTime: 2024-08-01 23:03:04
 */
#include "parser.h"
#include "path_finder_algorithm/dijkstra.h"

namespace map_parser {

MapParser::MapParser(ros::NodeHandle *nh) : nh_(nh) {
  nh_->param<std::string>("map_path", xml_file_name_, "test_route.xml");

  path_pub = nh_->advertise<nav_msgs::Path>("/route_network", 10);
  text_pub = nh_->advertise<visualization_msgs::MarkerArray>("/point_text", 10);
  map_point_pub =
      nh_->advertise<visualization_msgs::MarkerArray>("/map_point_marker", 10);

  map_pub = nh_->advertise<visualization_msgs::MarkerArray>("/map_marker", 10);

  all_route_paths_pub = nh_->advertise<visualization_msgs::MarkerArray>(
      "/all_route_paths_marker", 10);

  std::cout << xml_file_name_ << std::endl;
}

MapParser::MapParser() {}

void MapParser::set_new_map(std::string xml_name) {
  this->xml_file_name_ = xml_name;
}

void MapParser::set_type_of_path(std::vector<std::string> types) {
  this->type_of_path_ = types;
}

void MapParser::parse(bool onlyFirstElement) {
  TiXmlDocument doc(xml_file_name_);
  TiXmlNode *xml_map;
  TiXmlNode *node;
  TiXmlNode *path;

  TiXmlHandle hRootNode(0);
  TiXmlHandle hRootPath(0);

  bool loadOkay = doc.LoadFile();
  if (loadOkay) {
    ROS_INFO("Map parser: loaded map: %s", xml_file_name_.c_str());
  } else {
    ROS_ERROR("Map parser: Failed to load file %s", xml_file_name_.c_str());
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
  std::cout << "MapParser Init Finished!!!" << std::endl;
}

void MapParser::createNodes(TiXmlHandle *hRootNode, bool onlyFirstElement) {
  map_nodes_.clear();

  TiXmlElement *nodeElement = hRootNode->Element();

  int node_id = 0;
  for (nodeElement; nodeElement;
       nodeElement = nodeElement->NextSiblingElement("point")) {

    MAP_NODE tmp_node;

    tmp_node.node_id = node_id++;
    tmp_node.name = nodeElement->Attribute("name");
    node_name_to_id_[tmp_node.name] = tmp_node.node_id;
    node_id_to_name_[tmp_node.node_id] = tmp_node.name;
    nodeElement->Attribute("xPosition", &(tmp_node.x_pos));
    nodeElement->Attribute("yPosition", &(tmp_node.y_pos));
    nodeElement->Attribute("zPosition", &(tmp_node.z_pos));
    // 将单位从mm换算成m
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
      // 将单位从mm换算成m
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

  std::cout << "Point nums : " << map_nodes_.size() << std::endl;
}

void MapParser::createPaths(TiXmlHandle *hRootWay, bool onlyFirstElement) {
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
    // 将单位转换为m或者m/s
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
          // 将单位转换为m
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

  std::cout << "Path nums : " << map_paths_.size() << std::endl;

  // std::cout << "path[0] : " << map_paths_[0].path_id << ", "
  //           << map_paths_[0].source_node_name << ", "
  //           << map_paths_[0].destination_node_name << ", "
  //           << map_paths_[0].length << ", " << map_paths_[0].max_vel << ", "
  //           << map_paths_[0].max_reverse_vel << ", " << map_paths_[0].locked
  //           << ", " << map_paths_[0].path_layout.connection_type << ", "
  //           << map_paths_[0].path_layout.layer_id << ", "
  //           << map_paths_[0].path_layout.control_points[0].x << ", "
  //           << map_paths_[0].path_layout.control_points[0].y << ", "
  //           << map_paths_[0].path_layout.control_points[1].x << ", "
  //           << map_paths_[0].path_layout.control_points[1].y << std::endl;
}

void MapParser::createNetwork() {

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

std::shared_ptr<std::vector<std::vector<float>>> MapParser::getGraphOfVertex() {
  return std::make_shared<std::vector<std::vector<float>>>(networkArray_);
}

// TODO spravit const ref
void MapParser::publishPoint(geometry_msgs::Point point, int marker_type,
                             double radius,
                             geometry_msgs::Quaternion orientation) {
  point.z = 0;

  switch (marker_type) {
  case CURRENT_POSITION_MARKER:
    position_marker.points.clear();
    position_marker.scale.x = radius;
    position_marker.scale.y = radius;
    position_marker.pose.position = point;
    position_marker_pub.publish(position_marker);
    break;

  case TARGET_POSITION_MARKER:
    target_marker.points.clear();
    target_marker.scale.x = radius;
    target_marker.scale.y = radius;
    target_marker.pose.position = point;
    target_marker.pose.orientation = orientation;
    target_marker_pub.publish(target_marker);
    break;
  default:
    break;
  }
}

void MapParser::publishPoint(double latitude, double longitude, int marker_type,
                             double radius,
                             geometry_msgs::Quaternion orientation) {
  geometry_msgs::Point point;
  point.x = 0;
  point.y = 0;

  publishPoint(point, marker_type, radius, orientation);
}

void MapParser::publishPoint(int pointID, int marker_type, double radius,
                             geometry_msgs::Quaternion orientation) {
  geometry_msgs::Point point;
  point.x = map_nodes_[pointID].x_pos;
  point.y = map_nodes_[pointID].y_pos;
  publishPoint(point, marker_type, radius, orientation);
}

void MapParser::publishPoint(const MAP_NODE &node, int marker_type,
                             double radius,
                             geometry_msgs::Quaternion orientation) {
  geometry_msgs::Point point;
  point.x = node.x_pos;
  point.y = node.y_pos;
  publishPoint(point, marker_type, radius, orientation);
}

// publishing all paths
void MapParser::publishRouteNetwork() {
  nav_msgs::Path path;
  path.header.frame_id = "world";

  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = 0;
  pose.pose.position.y = 0;
  pose.pose.position.z = 0;

  for (int i = 0; i < map_paths_.size(); i++) {
    path.poses.clear();
    pose.pose.position.x =
        map_nodes_[node_name_to_id_[map_paths_[i].source_node_name]].x_pos;
    pose.pose.position.y =
        map_nodes_[node_name_to_id_[map_paths_[i].source_node_name]].y_pos;
    path.poses.push_back(pose);
    std::cout << "source(" << pose.pose.position.x << ", "
              << pose.pose.position.y << ")" << std::endl;

    pose.pose.position.x =
        map_nodes_[node_name_to_id_[map_paths_[i].destination_node_name]].x_pos;
    pose.pose.position.y =
        map_nodes_[node_name_to_id_[map_paths_[i].destination_node_name]].y_pos;
    path.poses.push_back(pose);
    std::cout << "destination(" << pose.pose.position.x << ", "
              << pose.pose.position.y << ")" << std::endl;

    path.header.stamp = ros::Time::now();
    path_pub.publish(path);
  }
  std::cout << "paths_nums : " << map_paths_.size() << std::endl;
}

void MapParser::publishMapArrow() {
  int id_count = 0;
  map_point_markers.markers.clear();
  visualization_msgs::Marker point_marker;
  point_marker.header.frame_id = "world";
  point_marker.header.stamp = ros::Time::now();
  point_marker.ns = "map_points";
  point_marker.action = visualization_msgs::Marker::ADD;
  point_marker.type = visualization_msgs::Marker::ARROW;
  point_marker.lifetime = ros::Duration(0);
  // Scale
  point_marker.scale.x = 1.0;
  point_marker.scale.y = 0.3;
  point_marker.scale.z = 0.3;
  // Color
  point_marker.color.r = 1.0;
  point_marker.color.g = 0.0;
  point_marker.color.b = 0.0;
  point_marker.color.a = 1.0;
  for (int i = 0; i < map_paths_.size(); i++) {
    point_marker.id = id_count++;

    double start_x =
        map_nodes_[node_name_to_id_[map_paths_[i].source_node_name]].x_pos;
    double start_y =
        map_nodes_[node_name_to_id_[map_paths_[i].source_node_name]].y_pos;
    double end_x =
        map_nodes_[node_name_to_id_[map_paths_[i].destination_node_name]].x_pos;
    double end_y =
        map_nodes_[node_name_to_id_[map_paths_[i].destination_node_name]].y_pos;
    double yaw = atan2(end_y - start_y, end_x - start_x);
    auto tf_q = tf::createQuaternionFromYaw(yaw);
    point_marker.pose.orientation.x = tf_q.getX();
    point_marker.pose.orientation.y = tf_q.getY();
    point_marker.pose.orientation.z = tf_q.getZ();
    point_marker.pose.orientation.w = tf_q.getW();

    point_marker.pose.position.x = start_x;
    point_marker.pose.position.y = start_y;
    point_marker.pose.position.z = 0.0;

    map_point_markers.markers.emplace_back(point_marker);
  }
  map_point_pub.publish(map_point_markers);

  publishPointText();
}

void MapParser::publishMapPoint() {
  int id_count = 0;
  map_point_markers.markers.clear();
  visualization_msgs::Marker point_marker;
  point_marker.header.frame_id = "world";
  point_marker.header.stamp = ros::Time::now();
  point_marker.ns = "map_points";
  point_marker.action = visualization_msgs::Marker::ADD;
  point_marker.type = visualization_msgs::Marker::CYLINDER;
  point_marker.lifetime = ros::Duration(0);
  // Scale
  point_marker.scale.x = 0.5;
  point_marker.scale.y = 0.5;
  point_marker.scale.z = 0.5;
  // Color
  point_marker.color.r = 1.0;
  point_marker.color.g = 0.0;
  point_marker.color.b = 0.0;
  point_marker.color.a = 1.0;
  for (int i = 0; i < map_nodes_.size(); i++) {

    point_marker.id = id_count++;
    point_marker.pose.position.x = map_nodes_[i].x_pos;
    point_marker.pose.position.y = map_nodes_[i].y_pos;
    point_marker.pose.position.z = 0.0;
    map_point_markers.markers.emplace_back(point_marker);
  }
  map_point_pub.publish(map_point_markers);
}

void MapParser::publishPointText() {
  int id_count = 0;
  point_texts.markers.clear();
  visualization_msgs::Marker point_marker;
  point_marker.header.frame_id = "world";
  point_marker.header.stamp = ros::Time::now();
  point_marker.ns = "points_text";
  point_marker.action = visualization_msgs::Marker::ADD;
  point_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  point_marker.lifetime = ros::Duration(0);
  // Scale
  point_marker.scale.x = 0.5;
  point_marker.scale.y = 0.5;
  point_marker.scale.z = 0.5;
  // Color
  point_marker.color.r = 1.0;
  point_marker.color.g = 1.0;
  point_marker.color.b = 1.0;
  point_marker.color.a = 1.0;
  for (int i = 0; i < map_nodes_.size(); i++) {

    point_marker.id = id_count++;
    point_marker.pose.position.x = map_nodes_[i].x_pos;
    point_marker.pose.position.y = map_nodes_[i].y_pos + 0.6;
    point_marker.pose.position.z = 0.0;
    point_marker.text = map_nodes_[i].name;
    point_texts.markers.emplace_back(point_marker);
  }
  text_pub.publish(point_texts);
}

void MapParser::publishMapArray() {
  int id_count = 0;
  map_marker_array.markers.clear();
  // draw center line
  for (int i = 0; i < map_paths_.size(); i++) {
    drawCenterLine(map_paths_[i], id_count);
    id_count++;
  }

  // 画停车库

  visualization_msgs::Marker point_marker;
  point_marker.header.frame_id = "world";
  point_marker.header.stamp = ros::Time::now();
  point_marker.ns = "map_garage";
  point_marker.action = visualization_msgs::Marker::ADD;
  point_marker.type = visualization_msgs::Marker::CUBE;
  point_marker.id = id_count++;
  point_marker.lifetime = ros::Duration(0);
  // Scale
  point_marker.scale.x = 4.5;
  point_marker.scale.y = 4.5;
  point_marker.scale.z = 0.5;
  // Color
  point_marker.color.r = 0.0;
  point_marker.color.g = 1.0;
  point_marker.color.b = 0.0;
  point_marker.color.a = 0.5;

  point_marker.pose.position.x = map_nodes_.back().x_pos;
  point_marker.pose.position.y = map_nodes_.back().y_pos;
  map_marker_array.markers.emplace_back(point_marker);

  point_marker.id = id_count++;
  point_marker.pose.position.x = map_nodes_[map_nodes_.size() - 2].x_pos;
  point_marker.pose.position.y = map_nodes_[map_nodes_.size() - 2].y_pos;
  map_marker_array.markers.emplace_back(point_marker);

  map_pub.publish(map_marker_array);
}

void MapParser::drawCenterLine(const MapParser::MAP_PATH &map_path,
                               int &count_id) {
  visualization_msgs::Marker map_line;
  map_line.header.frame_id = "world";
  map_line.header.seq = count_id;
  map_line.header.stamp = ros::Time::now();
  map_line.ns = "map_points";
  map_line.action = visualization_msgs::Marker::ADD;
  map_line.type = visualization_msgs::Marker::LINE_STRIP;
  map_line.id = count_id;
  map_line.lifetime = ros::Duration(0);
  // Scale
  map_line.scale.x = 0.3;
  map_line.scale.y = 0.3;
  map_line.scale.z = 0.3;
  if (count_id % 3 == 0) {
    int scale = count_id / 3;
    map_line.color.r = 0.14 * scale;
    map_line.color.g = 0.0;
    map_line.color.b = 0.0;
  } else if (count_id % 3 == 1) {
    int scale = count_id / 3;
    map_line.color.r = 0.0;
    map_line.color.g = 0.14 * scale;
    map_line.color.b = 0.0;
  } else {
    int scale = count_id / 3;
    map_line.color.r = 0.0;
    map_line.color.g = 0.0;
    map_line.color.b = 0.14 * scale;
  }
  map_line.color.r = 0.0;
  map_line.color.g = 1.0;
  map_line.color.b = 0.0;
  map_line.color.a = 0.3;

  geometry_msgs::Point temp_point;

  temp_point.x = map_nodes_[node_name_to_id_[map_path.source_node_name]].x_pos;
  temp_point.y = map_nodes_[node_name_to_id_[map_path.source_node_name]].y_pos;
  map_line.points.push_back(temp_point);

  temp_point.x =
      map_nodes_[node_name_to_id_[map_path.destination_node_name]].x_pos;
  temp_point.y =
      map_nodes_[node_name_to_id_[map_path.destination_node_name]].y_pos;
  map_line.points.push_back(temp_point);

  map_marker_array.markers.push_back(map_line);
}

void MapParser::drawOneRoutePath(const std::vector<int> &path_id,
                                 int &count_id) {
  visualization_msgs::Marker path_line;
  path_line.header.frame_id = "world";
  path_line.header.seq = count_id;
  path_line.header.stamp = ros::Time::now();
  path_line.ns = "route_path_points";
  path_line.action = visualization_msgs::Marker::ADD;
  path_line.type = visualization_msgs::Marker::LINE_STRIP;
  path_line.id = count_id;
  path_line.lifetime = ros::Duration(0);
  // Scale
  path_line.scale.x = 0.5;
  path_line.scale.y = 0.5;
  path_line.scale.z = 0.5;
  if (count_id % 3 == 0) {
    int scale = count_id / 3;
    path_line.color.r = 1.0;
    path_line.color.g = 0.0;
    path_line.color.b = 0.0;
    path_line.scale.x = 1.0;
    path_line.scale.y = 1.0;
  } else if (count_id % 3 == 1) {
    int scale = count_id / 3;
    path_line.color.r = 0.0;
    path_line.color.g = 0.0;
    path_line.color.b = 1.0;
    path_line.scale.x = 1.0;
    path_line.scale.y = 1.0;
  } else {
    int scale = count_id / 3;
    path_line.color.r = 0.0;
    path_line.color.g = 1.0;
    path_line.color.b = 1.0;
    path_line.scale.x = 1.0;
    path_line.scale.y = 1.0;
  }
  // path_line.color.r = 1.0;
  // path_line.color.g = 1.0;
  // path_line.color.b = 0.0;
  path_line.color.a = 0.5;

  geometry_msgs::Point temp_point;

  for (int i = 0; i < path_id.size(); i++) {
    temp_point.x = map_nodes_[path_id[i]].x_pos;
    temp_point.y = map_nodes_[path_id[i]].y_pos;
    path_line.points.push_back(temp_point);
  }

  all_route_paths_markers.markers.push_back(path_line);
}

void MapParser::drawOneRoutePath(
    const std::vector<std::pair<double, double>> &one_path, int &count_id) {
  visualization_msgs::Marker path_line;
  path_line.header.frame_id = "world";
  path_line.header.seq = count_id;
  path_line.header.stamp = ros::Time::now();
  path_line.ns = "route_path_points";
  path_line.action = visualization_msgs::Marker::ADD;
  path_line.type = visualization_msgs::Marker::LINE_STRIP;
  path_line.id = count_id;
  path_line.lifetime = ros::Duration(0.5);
  // Scale
  path_line.scale.x = 0.5;
  path_line.scale.y = 0.5;
  path_line.scale.z = 0.5;
  if (count_id % 3 == 0) {
    int scale = count_id / 3;
    path_line.color.r = 1.0;
    path_line.color.g = 0.0;
    path_line.color.b = 0.0;
    path_line.scale.x = 1.0;
    path_line.scale.y = 1.0;
  } else if (count_id % 3 == 1) {
    int scale = count_id / 3;
    path_line.color.r = 0.0;
    path_line.color.g = 0.0;
    path_line.color.b = 1.0;
    path_line.scale.x = 1.0;
    path_line.scale.y = 1.0;
  } else {
    int scale = count_id / 3;
    path_line.color.r = 0.0;
    path_line.color.g = 1.0;
    path_line.color.b = 1.0;
    path_line.scale.x = 1.0;
    path_line.scale.y = 1.0;
  }
  // path_line.color.r = 1.0;
  // path_line.color.g = 1.0;
  // path_line.color.b = 0.0;
  path_line.color.a = 0.5;

  geometry_msgs::Point temp_point;

  for (int i = 0; i < one_path.size(); i++) {
    temp_point.x = one_path[i].first;
    temp_point.y = one_path[i].second;
    path_line.points.push_back(temp_point);
  }

  all_route_paths_markers.markers.push_back(path_line);
}

void MapParser::publishAllRoutePath(
    const std::vector<std::vector<int>> &all_path_ids) {

  int id_count = 0;
  all_route_paths_markers.markers.clear();
  for (int i = 0; i < all_path_ids.size(); i++) {
    drawOneRoutePath(all_path_ids[i], id_count);
    id_count++;
  }
  all_route_paths_pub.publish(all_route_paths_markers);
}

void MapParser::publishAllRoutePath(
    const std::vector<std::vector<std::pair<double, double>>> &route_paths) {
  int id_count = 0;
  all_route_paths_markers.markers.clear();
  for (int i = 0; i < route_paths.size(); ++i) {
    drawOneRoutePath(route_paths[i], id_count);
    id_count++;
  }
  all_route_paths_pub.publish(all_route_paths_markers);
}

void MapParser::publishRouteNetwork(const std::vector<int> &route_paths_id) {
  nav_msgs::Path path;
  path.header.frame_id = "world";

  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = 0;
  pose.pose.position.y = 0;
  pose.pose.position.z = 0;

  path.poses.clear();
  for (int i = 0; i < route_paths_id.size(); i++) {
    pose.pose.position.x = map_nodes_[route_paths_id[i]].x_pos;
    pose.pose.position.y = map_nodes_[route_paths_id[i]].y_pos;
    path.poses.push_back(pose);
  }
  path.header.stamp = ros::Time::now();
  path_pub.publish(path);
}

} // namespace map_parser