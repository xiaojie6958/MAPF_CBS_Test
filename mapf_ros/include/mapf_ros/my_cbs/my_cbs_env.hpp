/*
 * @Descripttion:
 * @version:
 * @Author: CyberC3
 * @Date: 2024-04-06 22:58:22
 * @LastEditors: zhu-hu
 * @LastEditTime: 2024-04-21 23:06:31
 */
#pragma once

#include "../utils/neighbor.hpp"
#include "../utils/planresult.hpp"
#include "../utils/utility.hpp"
#include "parser.h"

using namespace map_parser;
using mapf::Neighbor;
using mapf::PlanResult;

struct State {
  State(double time, double x, double y, int id)
      : time(time), x(x), y(y), id(id) {}
  State() : time(0), x(0), y(0), id(0) {}

  bool operator==(const State &s) const {
    return time == s.time && x == s.x && y == s.y;
  }

  bool equalExceptTime(const State &s) const { return x == s.x && y == s.y; }

  friend std::ostream &operator<<(std::ostream &os, const State &s) {
    return os << s.time << ": (" << s.x << "," << s.y << ")";
    // return os << "(" << s.x << "," << s.y << ")";
  }

  double time;
  double x;
  double y;
  int id;
};

namespace std {
template <> struct hash<State> {
  size_t operator()(const State &s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.time);
    boost::hash_combine(seed, s.x);
    boost::hash_combine(seed, s.y);
    return seed;
  }
};
} // namespace std

///动作类型，运动、等待
enum class Action {
  Move,
  Wait,
};

std::ostream &operator<<(std::ostream &os, const Action &a) {
  switch (a) {
  case Action::Move:
    os << "Move";
    break;
  case Action::Wait:
    os << "Wait";
    break;
  }
  return os;
}

struct Conflict {
  //冲突类型：顶点冲突或者边冲突
  enum Type {
    Vertex,
    Edge,
  };
  //冲突的时间
  double time;
  //发生冲突的两个agent索引
  size_t agent1;
  size_t agent2;
  Type type;
  //发生冲突的位置，如果是顶点冲突，则x1和y1起作用；如果是边冲突，则需要两个点确定一条边
  double x1;
  double y1;
  int id1;
  double x2;
  double y2;
  int id2;

  friend std::ostream &operator<<(std::ostream &os, const Conflict &c) {
    switch (c.type) {
    case Vertex:
      return os << c.time << ": Vertex(" << c.x1 << "," << c.y1 << ")";
    case Edge:
      return os << c.time << ": Edge(" << c.x1 << "," << c.y1 << "," << c.x2
                << "," << c.y2 << ")";
    }
    return os;
  }
};
//顶点约束
struct VertexConstraint {
  VertexConstraint(double time, double x, double y) : time(time), x(x), y(y) {}
  double time;
  double x;
  double y;
  int id;

  bool operator<(const VertexConstraint &other) const {
    return std::tie(time, x, y) < std::tie(other.time, other.x, other.y);
  }

  bool operator==(const VertexConstraint &other) const {
    return std::tie(time, x, y) == std::tie(other.time, other.x, other.y);
  }

  friend std::ostream &operator<<(std::ostream &os, const VertexConstraint &c) {
    return os << "VC(" << c.time << "," << c.x << "," << c.y << ")";
  }
};
//顶点约束的hash函数全特化
namespace std {
template <> struct hash<VertexConstraint> {
  size_t operator()(const VertexConstraint &s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.time);
    boost::hash_combine(seed, s.x);
    boost::hash_combine(seed, s.y);
    return seed;
  }
};
} // namespace std
//边约束
struct EdgeConstraint {
  EdgeConstraint(double time, double x1, double y1, double x2, double y2)
      : time(time), x1(x1), y1(y1), x2(x2), y2(y2) {}
  double time;
  double x1;
  double y1;
  int id1;
  double x2;
  double y2;
  int id2;

  bool operator<(const EdgeConstraint &other) const {
    return std::tie(time, x1, y1, x2, y2) <
           std::tie(other.time, other.x1, other.y1, other.x2, other.y2);
  }

  bool operator==(const EdgeConstraint &other) const {
    return std::tie(time, x1, y1, x2, y2) ==
           std::tie(other.time, other.x1, other.y1, other.x2, other.y2);
  }

  friend std::ostream &operator<<(std::ostream &os, const EdgeConstraint &c) {
    return os << "EC(" << c.time << "," << c.x1 << "," << c.y1 << "," << c.x2
              << "," << c.y2 << ")";
  }
};
//边约束hash的全特化
namespace std {
template <> struct hash<EdgeConstraint> {
  size_t operator()(const EdgeConstraint &s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.time);
    boost::hash_combine(seed, s.x1);
    boost::hash_combine(seed, s.y1);
    boost::hash_combine(seed, s.x2);
    boost::hash_combine(seed, s.y2);
    return seed;
  }
};
} // namespace std
//约束的结构体
struct Constraints {
  std::unordered_set<VertexConstraint> vertexConstraints;
  std::unordered_set<EdgeConstraint> edgeConstraints;
  //添加约束
  void add(const Constraints &other) {
    vertexConstraints.insert(other.vertexConstraints.begin(),
                             other.vertexConstraints.end());
    edgeConstraints.insert(other.edgeConstraints.begin(),
                           other.edgeConstraints.end());
  }
  //判断两个约束是否有重合
  bool overlap(const Constraints &other) const {
    for (const auto &vc : vertexConstraints) {
      if (other.vertexConstraints.count(vc) > 0) {
        return true;
      }
    }
    for (const auto &ec : edgeConstraints) {
      if (other.edgeConstraints.count(ec) > 0) {
        return true;
      }
    }
    return false;
  }

  friend std::ostream &operator<<(std::ostream &os, const Constraints &c) {
    for (const auto &vc : c.vertexConstraints) {
      os << vc << std::endl;
    }
    for (const auto &ec : c.edgeConstraints) {
      os << ec << std::endl;
    }
    return os;
  }
};

struct Location {
  Location(double x, double y, int id) : x(x), y(y), id(id) {}
  double x;
  double y;
  int id;

  bool operator<(const Location &other) const {
    return std::tie(x, y) < std::tie(other.x, other.y);
  }

  bool operator==(const Location &other) const {
    return std::tie(x, y) == std::tie(other.x, other.y);
  }

  friend std::ostream &operator<<(std::ostream &os, const Location &c) {
    return os << "(" << c.x << "," << c.y << ")";
  }
};

namespace std {
template <> struct hash<Location> {
  size_t operator()(const Location &s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.x);
    boost::hash_combine(seed, s.y);
    return seed;
  }
};
} // namespace std

class Environment {
public:
  Environment(std::vector<Location> goals,
              std::vector<MapParser::MAP_NODE> nodes,
              std::unordered_map<std::string, int> node_name_to_id,
              std::unordered_map<int, std::string> node_id_to_name,
              std::vector<std::vector<float>> network_array)
      : m_goals_(std::move(goals)), map_nodes_(std::move(nodes)),
        node_name_to_id_(node_name_to_id), node_id_to_name_(node_id_to_name),
        original_network_array_(std::move(network_array)),
        m_constraints_(nullptr) {}

  Environment(const Environment &) = delete;
  Environment &operator=(const Environment &) = delete;

  // void setLowLevelContext(size_t agentIdx, const Constraints *constraints) {
  //   assert(constraints); // NOLINT
  //   m_constraints_ = constraints;
  //   m_lastGoalConstraint_ = -1;
  //   for (const auto &vc : constraints->vertexConstraints) {
  //     if (vc.x == m_goals_[m_agentIdx_].x && vc.y == m_goals_[m_agentIdx_].y)
  //     {
  //       m_lastGoalConstraint_ = std::max(m_lastGoalConstraint_, vc.time);
  //     }
  //   }
  // }

  // bool isSolution(const State &s) {
  //   return s.x == m_goals_[m_agentIdx_].x && s.y == m_goals_[m_agentIdx_].y
  //   &&
  //          s.time > m_lastGoalConstraint_;
  // }

  void getNeighbors(const State &s,
                    std::vector<Neighbor<State, Action, int>> &neighbors) {
    neighbors.clear();
    {
      State n(s.time + 1, s.x, s.y, -1);
      if (stateValid(n) && transitionValid(s, n)) {
        neighbors.emplace_back(
            Neighbor<State, Action, int>(n, Action::Wait, 1));
      }
    }
  }

  bool
  getFirstConflict(const std::vector<PlanResult<State, Action, int>> &solution,
                   Conflict &result) {
    int max_t = 0;
    for (const auto &sol : solution) {
      max_t = std::max<int>(max_t, sol.states.size() - 1);
    }

    for (int t = 0; t <= max_t; ++t) {
      // check drive-drive vertex collisions
      for (size_t i = 0; i < solution.size(); ++i) {
        State state1 = getState(i, solution, t);
        for (size_t j = i + 1; j < solution.size(); ++j) {
          State state2 = getState(j, solution, t);
          if (state1.equalExceptTime(state2)) {
            result.time = t;
            result.agent1 = i;
            result.agent2 = j;
            result.type = Conflict::Vertex;
            result.x1 = state1.x;
            result.y1 = state1.y;
            // std::cout << "VC " << t << "," << state1.x << "," << state1.y <<
            // std::endl;
            return true;
          }
        }
      }
      // drive-drive edge (swap)
      for (size_t i = 0; i < solution.size(); ++i) {
        State state1a = getState(i, solution, t);
        State state1b = getState(i, solution, t + 1);
        for (size_t j = i + 1; j < solution.size(); ++j) {
          State state2a = getState(j, solution, t);
          State state2b = getState(j, solution, t + 1);
          if (state1a.equalExceptTime(state2b) &&
              state1b.equalExceptTime(state2a)) {
            result.time = t;
            result.agent1 = i;
            result.agent2 = j;
            result.type = Conflict::Edge;
            result.x1 = state1a.x;
            result.y1 = state1a.y;
            result.x2 = state1b.x;
            result.y2 = state1b.y;
            return true;
          }
        }
      }
    }

    return false;
  }

  void
  createConstraintsFromConflict(const Conflict &conflict,
                                std::map<size_t, Constraints> &constraints) {
    if (conflict.type == Conflict::Vertex) {
      Constraints c1;
      c1.vertexConstraints.emplace(
          VertexConstraint(conflict.time, conflict.x1, conflict.y1));
      constraints[conflict.agent1] = c1;
      constraints[conflict.agent2] = c1;
    } else if (conflict.type == Conflict::Edge) {
      Constraints c1;
      c1.edgeConstraints.emplace(EdgeConstraint(
          conflict.time, conflict.x1, conflict.y1, conflict.x2, conflict.y2));
      constraints[conflict.agent1] = c1;
      Constraints c2;
      c2.edgeConstraints.emplace(EdgeConstraint(
          conflict.time, conflict.x2, conflict.y2, conflict.x1, conflict.y1));
      constraints[conflict.agent2] = c2;
    }
  }

  // void onExpandHighLevelNode(int /*cost*/) { m_highLevelExpanded_++; }

  // void onExpandLowLevelNode(const State & /*s*/, int /*fScore*/,
  //                           int /*gScore*/) {
  //   m_lowLevelExpanded_++;
  // }

  // int highLevelExpanded() { return m_highLevelExpanded_; }

  // int lowLevelExpanded() const { return m_lowLevelExpanded_; }

private:
  State getState(size_t agentIdx,
                 const std::vector<PlanResult<State, Action, int>> &solution,
                 size_t t) {
    assert(agentIdx < solution.size());
    if (t < solution[agentIdx].states.size()) {
      return solution[agentIdx].states[t].first;
    }
    assert(!solution[agentIdx].states.empty());
    // if (m_disappearAtGoal_) {
    //   // This is a trick to avoid changing the rest of the code significantly
    //   // After an agent disappeared, put it at a unique but invalid position
    //   // This will cause all calls to equalExceptTime(.) to return false.
    //   return State(-1, -1 * (agentIdx + 1), -1, -1);
    // }
    return solution[agentIdx].states.back().first;
  }

  bool stateValid(const State &s) {}

  bool transitionValid(const State &s1, const State &s2) {
    assert(m_constraints_);
    const auto &con = m_constraints_->edgeConstraints;
    return con.find(EdgeConstraint(s1.time, s1.x, s1.y, s2.x, s2.y)) ==
           con.end();
  }

private:
  // std::vector< std::vector<int> > m_heuristic;
  const Constraints *m_constraints_;
  // //上一个目标的限制，表征时间步长的一个量
  // double m_lastGoalConstraint_;
  // //上层扩展次数
  // int m_highLevelExpanded_;
  // //下层扩展次数
  // int m_lowLevelExpanded_;
  // bool m_disappearAtGoal_;

public:
  //智能体的索引
  // size_t m_agentIdx_;
  //多个目标点位置
  std::vector<Location> m_goals_;
  //地图中所有的node节点
  std::vector<MapParser::MAP_NODE> map_nodes_;
  // node节点由名字到id的转换map
  std::unordered_map<std::string, int> node_name_to_id_;
  // node节点由id到名字的转换map
  std::unordered_map<int, std::string> node_id_to_name_;
  //地图的拓扑结构
  std::vector<std::vector<float>> original_network_array_;
};