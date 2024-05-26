#pragma once

#include <map>

#include "../utils/timer.hpp"
#include "a_star.hpp"

namespace mapf {

/*!
  \example cbs.cpp Example that solves the Multi-Agent Path-Finding (MAPF)
  problem in a 2D grid world with up/down/left/right
  actions
*/

/*! \brief Conflict-Based-Search (CBS) algorithm to solve the Multi-Agent
Path-Finding (MAPF) problem
This class implements the Conflict-Based-Search (CBS) algorithm.
This algorithm can find collision-free path for multiple agents with start and
goal locations
given for each agent.
CBS is a two-level search. On the low-level, A* is used to find paths for
individual agents (ideally using a perfect heuristic).
The high-level is a tree-search that resolves conflicts between agents as they
occur, earliest conflict-time first.
CBS is optimal with respect to the sum-of-individual costs.
Details of the algorithm can be found in the following paper:\n
Guni Sharon, Roni Stern, Ariel Felner, Nathan R. Sturtevant:\n
"Conflict-based search for optimal multi-agent pathfinding". Artif. Intell. 219:
40-66 (2015)\n
https://doi.org/10.1016/j.artint.2014.11.006
The underlying A* can either use a fibonacci heap, or a d-ary heap.
The latter is the default. Define "USE_FIBONACCI_HEAP" to use the fibonacci heap
instead.
\tparam State Custom state for the search. Needs to be copy'able
\tparam Action Custom action for the search. Needs to be copy'able
\tparam Cost Custom Cost type (integer or floating point types)
\tparam Conflict Custom conflict description. A conflict needs to be able to be
transformed into a constraint.
\tparam Constraints Custom constraint description. The Environment needs to be
able to search on the low-level while taking the constraints into account.
\tparam Environment This class needs to provide the custom logic. In particular,
it needs to support the following functions:
  - `void setLowLevelContext(size_t agentIdx, const Constraints* constraints)`\n
    Set the current context to a particular agent with the given set of
constraints
  - `Cost admissibleHeuristic(const State& s)`\n
    Admissible heuristic. Needs to take current context into account.
  - `bool isSolution(const State& s)`\n
    Return true if the given state is a goal state for the current agent.
  - `void getNeighbors(const State& s, std::vector<Neighbor<State, Action, int>
>& neighbors)`\n
    Fill the list of neighboring state for the given state s and the current
agent.
  - `bool getFirstConflict(const std::vector<PlanResult<State, Action, int> >&
solution, Conflict& result)`\n
    Finds the first conflict for the given solution for each agent. Return true
if a conflict was found and false otherwise.
  - `void createConstraintsFromConflict(const Conflict& conflict,
std::map<size_t, Constraints>& constraints)`\n
    Create a list of constraints for the given conflict.
  - `void onExpandHighLevelNode(Cost cost)`\n
    This function is called on every high-level expansion and can be used for
statistical purposes.
  - `void onExpandLowLevelNode(const State& s, Cost fScore, Cost gScore)`\n
    This function is called on every low-level expansion and can be used for
statistical purposes.
*/
template <typename State, typename Action, typename Cost, typename Conflict,
          typename Constraints, typename Environment>
class CBS {
public:
  CBS(Environment &environment) : m_env(environment) {}

  bool search(const std::vector<State> &initialStates,
              std::vector<PlanResult<State, Action, Cost>> &solution,
              const double &time_tolerance) {
    Timer timer;
    //上层规划的一个初始值，即暂不考虑各个底层规划的冲突
    HighLevelNode start;
    start.solution.resize(initialStates.size());
    start.constraints.resize(initialStates.size());
    start.cost = 0;
    start.id = 0;

    for (size_t i = 0; i < initialStates.size(); ++i) {
      // if (   i < solution.size()
      //     && solution[i].states.size() > 1) {
      //   start.solution[i] = solution[i];
      //   std::cout << "use existing solution for agent: " << i << std::endl;
      // } else {
      LowLevelEnvironment llenv(m_env, i, start.constraints[i]);
      LowLevelSearch_t lowLevel(llenv);
      bool success = lowLevel.search(initialStates[i], start.solution[i], timer,
                                     time_tolerance);
      if (!success) {
        return false;
      }
      // }
      start.cost += start.solution[i].cost;
    }

    // std::priority_queue<HighLevelNode> open;
    //定义了一个二叉堆，该堆用于存储HighLevelNode类型的对象，并且该堆是可变的。排序依据是cost（重载"<""的方式可以看出)
    typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>,
                                     boost::heap::mutable_<true>>
        open;

    auto handle = open.push(start);
    (*handle).handle = handle;

    solution.clear();
    int id = 1;
    while (!open.empty()) {
      //弹出代价最小的上层规划结果
      HighLevelNode P = open.top();
      //上层规划的次数增加1
      m_env.onExpandHighLevelNode(P.cost);
      // std::cout << "expand: " << P << std::endl;

      open.pop();

      Conflict conflict;
      //判断下层规划中是否有冲突，如果有，找出第一个冲突，如果没有，规划结束，当前的上层规划结果即为全局结果
      if (!m_env.getFirstConflict(P.solution, conflict)) {
        // std::cout << "done; cost: " << P.cost << std::endl;
        solution = P.solution;
        return true;
      }

      // create additional nodes to resolve conflict
      // std::cout << "Found conflict: " << conflict << std::endl;
      // std::cout << "Found conflict at t=" << conflict.time << " type: " <<
      // conflict.type << std::endl;

      std::map<size_t, Constraints> constraints;
      //从冲突中构建新的限制条件
      m_env.createConstraintsFromConflict(conflict, constraints);
      for (const auto &c : constraints) {
        // std::cout << "Add HL node for " << c.first << std::endl;
        size_t i = c.first;
        // std::cout << "create child with id " << id << std::endl;
        HighLevelNode newNode = P;
        newNode.id = id;
        // (optional) check that this constraint was not included already
        // std::cout << newNode.constraints[i] << std::endl;
        // std::cout << c.second << std::endl;
        //对于新的限制约束条件，断言新的上层规划的限制条件与新建的限制条件没有重叠
        assert(!newNode.constraints[i].overlap(c.second));
        //在新的上层规划的限制条件中增加新的限制条件
        newNode.constraints[i].add(c.second);
        //总的代价减去产生冲突的机器人的原有代价
        newNode.cost -= newNode.solution[i].cost;
        //在增加了新的限制后，对冲突发生改变的机器人进行新的下层规划
        LowLevelEnvironment llenv(m_env, i, newNode.constraints[i]);
        LowLevelSearch_t lowLevel(llenv);
        bool success = lowLevel.search(initialStates[i], newNode.solution[i],
                                       timer, time_tolerance);

        newNode.cost += newNode.solution[i].cost;
        //如果下层规划成功，将新的上层规划节点纳入考查范围
        if (success) {
          // std::cout << "  success. cost: " << newNode.cost << std::endl;
          auto handle = open.push(newNode);
          (*handle).handle = handle;
        }

        ++id;

        // check time tolerance
        timer.stop();
        if (timer.elapsedSeconds() > time_tolerance) {
          return false;
        }
      }
    }

    return false;
  }

private:
  struct HighLevelNode {
    //每一个机器人的规划结果存储
    std::vector<PlanResult<State, Action, Cost>> solution;
    //每一个机器人的限制条件约束
    std::vector<Constraints> constraints;
    //上层规划的总代价
    Cost cost;
    //上层规划节点的id
    int id;

    typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>,
                                     boost::heap::mutable_<true>>::handle_type
        handle;

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
    LowLevelEnvironment(Environment &env, size_t agentIdx,
                        const Constraints &constraints)
        : m_env(env)
    // , m_agentIdx(agentIdx)
    // , m_constraints(constraints)
    {
      m_env.setLowLevelContext(agentIdx, &constraints);
    }

    Cost admissibleHeuristic(const State &s) {
      return m_env.admissibleHeuristic(s);
    }

    bool isSolution(const State &s) { return m_env.isSolution(s); }

    void getNeighbors(const State &s,
                      std::vector<Neighbor<State, Action, Cost>> &neighbors) {
      m_env.getNeighbors(s, neighbors);
    }

    void onExpandNode(const State &s, Cost fScore, Cost gScore) {
      // std::cout << "LL expand: " << s << std::endl;
      m_env.onExpandLowLevelNode(s, fScore, gScore);
    }

    void onDiscover(const State & /*s*/, Cost /*fScore*/, Cost /*gScore*/) {
      // std::cout << "LL discover: " << s << std::endl;
      // m_env.onDiscoverLowLevel(s, m_agentIdx, m_constraints);
    }

  private:
    Environment &m_env;
    // size_t m_agentIdx;
    // const Constraints& m_constraints;
  };

private:
  Environment &m_env;
  typedef AStar<State, Action, Cost, LowLevelEnvironment> LowLevelSearch_t;
};

} // namespace mapf