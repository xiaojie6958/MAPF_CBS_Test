/*
 * @Descripttion:
 * @version:
 * @Author: CyberC3
 * @Date: 2024-04-06 22:59:17
 * @LastEditors: zhu-hu
 * @LastEditTime: 2024-04-22 00:04:28
 */
#pragma once

#include "../utils/neighbor.hpp"
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

    // std::priority_queue<HighLevelNode> open;
    //定义了一个二叉堆，该堆用于存储HighLevelNode类型的对象，并且该堆是可变的。排序依据是cost（重载"<""的方式可以看出)
    // typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>,
    //                                  boost::heap::mutable_<true>>
    //     open;

    // auto handle = open.push(start);
    // (*handle).handle = handle;

    // solution.clear();
    // int id = 1;
    // while (!open.empty()) {
    //   //弹出代价最小的上层规划结果
    //   HighLevelNode P = open.top();
    //   //上层规划的次数增加1
    //   m_env.onExpandHighLevelNode(P.cost);
    //   // std::cout << "expand: " << P << std::endl;

    //   open.pop();

    //   Conflict conflict;
    //   //判断下层规划中是否有冲突，如果有，找出第一个冲突，如果没有，规划结束，当前的上层规划结果即为全局结果
    //   if (!m_env.getFirstConflict(P.solution, conflict)) {
    //     // std::cout << "done; cost: " << P.cost << std::endl;
    //     solution = P.solution;
    //     return true;
    //   }

    //   std::map<size_t, Constraints> constraints;
    //   //从冲突中构建新的限制条件
    //   m_env.createConstraintsFromConflict(conflict, constraints);
    //   for (const auto &c : constraints) {
    //     // std::cout << "Add HL node for " << c.first << std::endl;
    //     size_t i = c.first;
    //     // std::cout << "create child with id " << id << std::endl;
    //     HighLevelNode newNode = P;
    //     newNode.id = id;

    //     //对于新的限制约束条件，断言新的上层规划的限制条件与新建的限制条件没有重叠
    //     assert(!newNode.constraints[i].overlap(c.second));
    //     //在新的上层规划的限制条件中增加新的限制条件
    //     newNode.constraints[i].add(c.second);
    //     //总的代价减去产生冲突的机器人的原有代价
    //     newNode.cost -= newNode.solution[i].cost;
    //     //在增加了新的限制后，对冲突发生改变的机器人进行新的下层规划
    //     LowLevelEnvironment llenv(m_env, i, newNode.constraints[i]);
    //     LowLevelSearch_t lowLevel(llenv);
    //     bool success = lowLevel.search(initialStates[i], newNode.solution[i],
    //                                    timer, time_tolerance);

    //     newNode.cost += newNode.solution[i].cost;
    //     //如果下层规划成功，将新的上层规划节点纳入考查范围
    //     if (success) {
    //       // std::cout << "  success. cost: " << newNode.cost << std::endl;
    //       auto handle = open.push(newNode);
    //       (*handle).handle = handle;
    //     }

    //     ++id;

    //     // check time tolerance
    //     timer.stop();
    //     if (timer.elapsedSeconds() > time_tolerance) {
    //       return false;
    //     }
    //   }
    // }

    return true;
  }

private:
  struct HighLevelNode {
    //每一个机器人的规划结果存储
    std::vector<PlanResult<State, Action, Cost>> solution;
    //每一个机器人的限制条件约束
    std::vector<Constraints> constraints;
    //上层规划的总代价
    Cost cost;
    //上层规划节点的id好
    int id;

    // typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>,
    //                                  boost::heap::mutable_<true>>::handle_type
    //     handle;

    bool operator<(const HighLevelNode &n) const {
      // if (cost != n.cost)
      return cost > n.cost;
      // return id > n.id;
    }

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
    // , m_agentIdx(agentIdx)
    // , m_constraints(constraints)
    {
      //   m_env.setLowLevelContext(agentIdx, &constraints);
    }

    // bool isSolution(const State &s) { return m_env.isSolution(s); }

    // void getNeighbors(const State &s,
    //                   std::vector<Neighbor<State, Action, Cost>> &neighbors)
    //                   {
    //   m_env.getNeighbors(s, neighbors);
    // }

    // void onExpandNode(const State &s, Cost fScore, Cost gScore) {
    //   // std::cout << "LL expand: " << s << std::endl;
    //   m_env.onExpandLowLevelNode(s, fScore, gScore);
    // }

    // void onDiscover(const State & /*s*/, Cost /*fScore*/, Cost /*gScore*/) {
    //   // std::cout << "LL discover: " << s << std::endl;
    //   // m_env.onDiscoverLowLevel(s, m_agentIdx, m_constraints);
    // }

    //   private:
    Environment &m_env;
    size_t m_agentIdx;
    // size_t m_agentIdx;
    // const Constraints& m_constraints;
  };

private:
  Environment &m_env;

  typedef Dijkstra<State, Action, Cost, LowLevelEnvironment> LowLevelSearch_t;
};
} // namespace mapf