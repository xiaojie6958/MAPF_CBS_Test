/*
 * @Descripttion:
 * @version:
 * @Author: CyberC3
 * @Date: 2024-04-06 23:00:10
 * @LastEditors: zhu-hu
 * @LastEditTime: 2024-04-22 11:22:45
 */
#pragma once

#include <boost/heap/d_ary_heap.hpp>
#include <unordered_map>
#include <unordered_set>

#include "../utils/neighbor.hpp"
#include "../utils/planresult.hpp"
#include "../utils/timer.hpp"

#include "path_finder_algorithm/dijkstra.h"

namespace mapf {
template <typename State, typename Action, typename Cost,
          typename LowEnvironment, typename StateHasher = std::hash<State>>
class Dijkstra {
public:
  Dijkstra(LowEnvironment &environment) : m_env_(environment) {}

  bool search(const State &startState,
              PlanResult<State, Action, Cost> &solution, Timer &timer,
              const double &time_tolerance, Cost initialCost = 0) {
    solution.states.clear();
    State start;
    start.id = startState.id;
    start.x = m_env_.m_env.map_nodes_[start.id].x_pos;
    start.y = m_env_.m_env.map_nodes_[start.id].y_pos;
    solution.states.push_back(std::make_pair<>(start, 0));
    solution.actions.clear();
    solution.cost = 0;

    std::shared_ptr<std::vector<std::vector<float>>> graph =
        std::make_shared<std::vector<std::vector<float>>>(
            m_env_.m_env.original_network_array_);

    int source_id = startState.id;
    int target_id = m_env_.m_env.m_goals_[m_env_.m_agentIdx].id;

    auto result = dijksta_.findShortestPath(graph, source_id, target_id);

    std::cout << "result_size : " << result.size() << std::endl;

    for (int i = 1; i < result.size(); i++) {
      State tmp;
      tmp.id = result[i];
      tmp.x = m_env_.m_env.map_nodes_[result[i]].x_pos;
      tmp.y = m_env_.m_env.map_nodes_[result[i]].y_pos;

      Cost tmp_cost =
          m_env_.m_env.original_network_array_[result[i - 1]][result[i]];

      solution.states.push_back(std::make_pair<>(tmp, tmp_cost));

      solution.actions.push_back(std::make_pair<>(Action::Move, tmp_cost));

      solution.cost += tmp_cost;
    }
    return true;
  }

private:
  LowEnvironment &m_env_;
  map_parser::path_finder_algorithm::Dijkstra dijksta_;
};
} // namespace mapf