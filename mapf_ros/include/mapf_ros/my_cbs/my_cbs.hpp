/*
 * @Descripttion:
 * @version:
 * @Author: CyberC3
 * @Date: 2024-04-06 22:59:17
 * @LastEditors: zhu-hu
 * @LastEditTime: 2024-08-01 22:59:47
 */
#pragma once

#include "../utils/planresult.hpp"
#include "../utils/timer.hpp"
#include <map>
#include <vector>

#include <boost/heap/d_ary_heap.hpp>
#include <unordered_map>
#include <unordered_set>

#include "dijkstra.hpp"

namespace mapf {

template <typename State, typename Action, typename Cost, typename Conflict,
          typename Constraints, typename Environment>
class MYCBS {
public:
  MYCBS(Environment &environment) : m_env(environment) {}

  bool search(const std::vector<State> &initialSTates,
              std::vector<PlanResult<State, Action, int>> &solution,
              const double &time_tolerance) {
    Timer timer;
    HighLevelNode start;
    start.solution.resize(initialSTates.size());
    start.constraints.resize(initialSTates.size());
    start.cost = 0;
    start.id = 0;

    for (size_t i = 0; i < initialSTates.size(); ++i) {

      LowLevelEnvironment llenv(m_env, i);
      LowLevelSearch_t LowLevel(llenv);

      bool success = LowLevel.search(initialSTates[i], start.solution[i], timer,
                                     time_tolerance);

      if (!success) {
        return false;
      }

      start.cost += start.solution[i].cost;
    }

    solution = std::move(start.solution);

    return true;
  }

private:
  struct HighLevelNode {
    // 每一个机器人的规划结果存储
    std::vector<PlanResult<State, Action, Cost>> solution;
    // 每一个机器人的限制条件约束
    std::vector<Constraints> constraints;
    // 上层规划的总代价
    Cost cost;
    // 上层规划节点的id号
    int id;

    bool operator<(const HighLevelNode &n) const { return cost > n.cost; }

    friend std::ostream &operator<<(std::ostream &os, const HighLevelNode &c) {
      os << "id: " << c.id << " cost: " << c.cost << std::endl;
      for (size_t i = 0; i < c.solution.size(); ++i) {
        os << "Agent: " << i << std::endl;
        os << " States:" << std::endl;
        for (size_t t = 0; t < c.solution[i].states.size(); ++t) {
          os << "  " << c.solution[i].states[t].first << std::endl;
        }
        os << " Constraints:" << std::endl;
        os << c.constraints[i];
        os << " cost: " << c.solution[i].cost << std::endl;
      }
      return os;
    }
  };

  struct LowLevelEnvironment {
    LowLevelEnvironment(Environment &env, size_t agentIdx)
        : m_env(env), m_agentIdx(agentIdx)

    {}

    Environment &m_env;
    size_t m_agentIdx;
  };

private:
  Environment &m_env;

  typedef Dijkstra<State, Action, Cost, LowLevelEnvironment> LowLevelSearch_t;
};
} // namespace mapf