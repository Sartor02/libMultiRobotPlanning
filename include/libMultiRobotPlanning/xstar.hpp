#pragma once

#include <cassert>
#include <map>

#include "a_star.hpp"

#ifdef USE_FIBONACCI_HEAP
#include <boost/heap/fibonacci_heap.hpp>
#endif

#include <boost/heap/d_ary_heap.hpp>
#include <unordered_map>
#include <unordered_set>
#include "cartesian_product.hpp"

namespace libMultiRobotPlanning {

/*!
  \example cbs.cpp Example that solves the Multi-Agent Path-Finding (MAPF)
  problem in a 2D grid world with up/down/left/right
  actions
*/

/*! \brief Expanding A* (X*) algorithm to solve the Multi-Agent
Path-Finding (MAPF) problem

This class implements the Expanding A* (X*) algorithm.


Preliminary details of the algorithm can be found in the following paper:\n
https://arxiv.org/pdf/1811.12598.pdf

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
          typename Window, typename Environment,
          typename StateHasher = std::hash<std::vector<State>>>
class XStar {
 private:
  struct LowLevelEnvironment {
    LowLevelEnvironment(Environment& env, size_t agentIdx) : m_env(env) {
      m_env.setLowLevelContext(agentIdx);
    }

    Cost admissibleHeuristic(const State& s) {
      return m_env.admissibleHeuristic(s);
    }

    bool isSolution(const State& s) { return m_env.isSolution(s); }

    void getNeighbors(const State& s,
                      std::vector<Neighbor<State, Action, Cost>>& neighbors) {
      m_env.getNeighbors(s, neighbors);
    }

    void onExpandNode(const State& s, Cost fScore, Cost gScore) {
      // std::cout << "LL expand: " << s << std::endl;
      m_env.onExpandLowLevelNode(s, fScore, gScore);
    }

    void onDiscover(const State& /*s*/, Cost /*fScore*/, Cost /*gScore*/) {
      // std::cout << "LL discover: " << s << std::endl;
      // m_env.onDiscoverLowLevel(s, m_agentIdx, m_constraints);
    }

   private:
    Environment& m_env;
    // size_t m_agentIdx;
  };

  Environment& m_env;

  using JointState_t = std::vector<State>;
  using JointAction_t = std::vector<Action>;
  using JointCost_t = std::vector<Cost>;
  using LowLevelSearch_t = AStar<State, Action, Cost, LowLevelEnvironment>;
  using IndividualPlan_t = PlanResult<State, Action, Cost>;
  using JointPlan_t = std::vector<IndividualPlan_t>;

  static void print(const JointPlan_t& p, std::ostream& os = std::cout) {
    for (size_t i = 0; i < p.size(); ++i) {
      const auto& e = p[i];
      os << "i: " << i << ' ';
      for (const auto& sa : e.states) {
        os << sa.first << ' ';
      }
      os << " | ";
    }
  }

  struct SearchState {
    //     JointPlan_t plan;

    bool operator==(const SearchState& o) const {
      //       return (plan == o.plan);
      return true;
    }

    friend std::ostream& operator<<(std::ostream& os, const SearchState& ps) {
      //       assert(ps.plan.empty());
      //       print(ps.plan, os);
      return os;
    }
  };

  struct WindowPlannerState {
    Window window;
    SearchState search_state;

    WindowPlannerState() = delete;
    explicit WindowPlannerState(const Window& window)
        : window(window), search_state() {}
    WindowPlannerState(const Window& window, const SearchState* search_state)
        : window(window), search_state(search_state) {}

    bool operator==(const WindowPlannerState& o) const {
      return (window == o.window) && (search_state == o.search_state);
    }

    WindowPlannerState merge(const WindowPlannerState& o) const {
      return WindowPlannerState(window.merge(o.window));
    }

    bool overlapping(const WindowPlannerState& other) const {
      // If no overlap in space, then they cannot be overlapping in paths.
      if (!window.intersects(other.window)) {
        return false;
      }

      // TODO(kvedder): Add more intelligent overlap detection conditions.

      // If they overlap in space and share agents, then they need to be merged.
      if (window.has_overlapping_agents(other.window)) {
        return true;
      }

      return false;

      // If they overlap in space but don't share agents and at least one
      // doesn't have a path to collide with the other, then they do not need to
      // be merged.
      //       if (search_state.plan.empty() || other.search_state.plan.empty())
      //       {
      //         return false;
      //       }

      std::cerr << "Needs to check if plans collide!\n";
      exit(0);
      return false;
    }

    friend std::ostream& operator<<(std::ostream& os,
                                    const WindowPlannerState& ws) {
      return os << "Window: " << ws.window
                << " Search state: " << ws.search_state;
    }
  };

  using WPS_t = WindowPlannerState;
  using WPSList_t = std::vector<WindowPlannerState>;

  bool recWAMPF(WPSList_t& windows, JointPlan_t& solution) {
    std::cout << "Starting recWAMPF\n";
    for (WPS_t& window : windows) {
      growAndReplanIn(window, solution);
      if (existsOverlapping(window, windows)) {
        planInOverlapWindows(window, windows, solution);
      }
    }
    std::cout << "Finished growAndReplanIn\n";

    Conflict result;
    while (m_env.getFirstConflict(solution, result)) {
      std::cout << "Found first conflict\n";
      std::cout << "Conflict: " << result << std::endl;
      WPS_t window = WindowPlannerState(m_env.createWindowFromConflict(result));
      std::cout << "Got first window\n";
      planInOverlapWindows(window, windows, solution);
    }

    std::cout << "Finished all conflicts\n";
    return true;
  }

  bool existsOverlapping(const WPS_t& given_w, const WPSList_t& list) const {
    for (const WPS_t& w : list) {
      if (&given_w == &w) {
        // Ignore self.
        continue;
      }
      if (given_w.overlapping(w)) {
        return true;
      }
    }
    return false;
  }

  void growAndReplanIn(WPS_t& window, JointPlan_t& solution) {}

  bool shouldQuit() { return true; }

  bool windowOverlapsWithOther(const WPS_t& window,
                               const WPSList_t& windows) const {
    return true;
  }

  bool planInOverlapWindows(WPS_t& given_window, WPSList_t& windows,
                            JointPlan_t& solution) {
    std::cout << "Number of windows: " << windows.size() << std::endl;
    bool planned_in = false;
    for (size_t i = 0; i < windows.size();) {
      std::cout << "i: " << i << '\n';
      const WPS_t& w = windows[i];
      if (&given_window == &w) {
        // Ignore self.
        continue;
      }

      if (given_window.overlapping(w)) {
        std::cout << "Merging with window " << i << '\n';
        given_window = given_window.merge(w);
        windows.erase(windows.begin() + i);
        planIn(given_window, solution);
        planned_in = true;
      } else {
        ++i;
      }
    }

    if (!planned_in) {
      planIn(given_window, solution);
    }
    std::cout << "Windows push back\n";
    windows.push_back(given_window);
    return true;
  }

  struct Node {
    Node(const JointState_t& state, Cost fScore, JointCost_t gScore)
        : state(state),
          fScore(fScore),
          gScoreSum(utils::sum(gScore)),
          gScore(gScore) {}

    bool operator<(const Node& other) const {
      // Sort order
      // 1. lowest fScore
      // 2. highest gScore

      // Our heap is a maximum heap, so we invert the comperator function here
      if (fScore != other.fScore) {
        return fScore > other.fScore;
      } else {
        return gScoreSum < other.gScoreSum;
      }
    }

    friend std::ostream& operator<<(std::ostream& os, const Node& node) {
      os << "state: " << node.state << " fScore: " << node.fScore
         << " gScore: " << node.gScoreSum;
      return os;
    }

    JointState_t state;

    Cost fScore;
    Cost gScoreSum;
    JointCost_t gScore;

    typename boost::heap::d_ary_heap<Node, boost::heap::arity<2>,
                                     boost::heap::mutable_<true>>::handle_type
        handle;
  };

  bool isColliding(const JointState_t& n, const JointState_t& c,
                   const JointAction_t& a) {
    assert(n.size() == c.size());
    for (size_t i = 0; i < n.size(); ++i) {
      for (size_t j = i + 1; j < n.size(); ++j) {
        if (n[i] == n[j]) {
          return true;
        }
      }
    }

    for (size_t i = 0; i < n.size(); ++i) {
      for (size_t j = i + 1; j < n.size(); ++j) {
        if (n[i].equalExceptTime(c[j]) && c[i].equalExceptTime(n[j])) {
          return true;
        }
      }
    }

    return false;
  }

  void verifyPlanInEndpoints(const JointState_t& start,
                             const JointCost_t& start_cost,
                             const JointState_t& goal,
                             const JointCost_t& goal_cost) const {
    assert(!start.empty());
    assert(!start_cost.empty());
    assert(start.size() == start_cost.size());
    for (size_t i = 0; i < start.size(); ++i) {
      assert(start.at(i).time == start.front().time);
      assert(start_cost.at(i) == start_cost.front());
    }

    for (size_t i = 0; i < goal.size(); ++i) {
      for (size_t j = i + 1; j < goal.size(); ++j) {
        assert(goal[i] != goal[j]);
      }
    }

    std::cout << "Starts: ";
    for (const auto& s : start) {
      std::cout << s << " ";
    }
    std::cout << std::endl;

    std::cout << "Goal: ";
    for (const auto& s : goal) {
      std::cout << s << " ";
    }
    std::cout << std::endl;
  }

  bool needsGrow(const JointState_t& goals) {
    for (size_t i = 0; i < goals.size(); ++i) {
      for (size_t j = i + 1; j < goals.size(); ++j) {
        if (goals[i].equalExceptTime(goals[j])) {
          return true;
        }
      }
    }
    return false;
  }

  bool planIn(WPS_t& window, JointPlan_t& solution) {
    std::cout << "Plan in: " << window.window << '\n';

    auto starts_goals = window.window.getStartsAndGoals(solution);
    std::pair<JointState_t, JointCost_t>& starts = starts_goals.first;
    JointState_t& goals = starts_goals.second.first;
    JointCost_t& goals_costs = starts_goals.second.second;

    while (needsGrow(goals)) {
      std::cout << "Needed to grow" << std::endl;
      window.window.grow();
      starts_goals = window.window.getStartsAndGoals(solution);
      starts = starts_goals.first;
      goals = starts_goals.second.first;
      goals_costs = starts_goals.second.second;
    }

    verifyPlanInEndpoints(starts.first, starts.second, goals, goals_costs);

    assert(starts.first.size() == goals.size());

    JointState_t current;

    using openset_t =
        typename boost::heap::d_ary_heap<Node, boost::heap::arity<2>,
                                         boost::heap::mutable_<true>>;
    using fibHeapHandle_t = typename openset_t::handle_type;

    openset_t openSet;
    std::unordered_map<JointState_t, fibHeapHandle_t, StateHasher> stateToHeap;
    std::unordered_set<JointState_t, StateHasher> closedSet;
    std::unordered_map<
        JointState_t,
        std::tuple<JointState_t, JointAction_t, JointCost_t, JointCost_t>,
        StateHasher>
        cameFrom;

    auto handle = openSet.push(
        Node(starts.first, m_env.admissibleJointHeuristic(starts.first, goals),
             starts.second));
    stateToHeap.insert(std::make_pair<>(starts.first, handle));
    (*handle).handle = handle;

    JointPlan_t window_solution(starts.first.size());

    while (!openSet.empty()) {
      Node current = openSet.top();
      //             std::cout << "Current: ";
      //             for (const auto& s : current.state) {
      //               std::cout << s << ' ';
      //             }
      //             std::cout << std::endl;
      m_env.onExpandNode(current.state, current.fScore, current.gScore);
      if (m_env.isJointSolution(current.state, goals)) {
        auto iter = cameFrom.find(current.state);
        while (iter != cameFrom.end()) {
          for (size_t i = 0; i < iter->first.size(); ++i) {
            auto& i_solution = window_solution.at(i);
            const State& s_i = iter->first.at(i);
            const Action& a_i = std::get<1>(iter->second).at(i);
            const Cost& c_i = std::get<2>(iter->second).at(i);
            const Cost& g_i = std::get<3>(iter->second).at(i);

            i_solution.states.push_back({s_i, g_i});
            i_solution.actions.push_back({a_i, c_i});
          }
          iter = cameFrom.find(std::get<0>(iter->second));
        }
        for (size_t i = 0; i < window_solution.size(); ++i) {
          auto& solution = window_solution[i];
          solution.states.push_back(
              std::make_pair<>(starts.first.at(i), starts.second.at(i)));
          std::reverse(solution.states.begin(), solution.states.end());
          std::reverse(solution.actions.begin(), solution.actions.end());
          solution.cost = utils::sum(current.gScore);
          solution.fmin = current.fScore;
        }

        std::cout << "Unwind done!\n";
        return insertWindowPath(window_solution, window.window, starts.second,
                                goals_costs, solution);
      }

      openSet.pop();
      stateToHeap.erase(current.state);
      closedSet.insert(current.state);

      auto cartesian_product = m_env.getInWindowJointWindowNeighbors(
          current.state, goals, window.window, solution);
      do {
        JointState_t neighbor_joint_state = starts.first;
        JointAction_t neighbor_joint_action;
        JointCost_t neighbor_joint_cost;
        neighbor_joint_action.resize(starts.first.size());
        neighbor_joint_cost.resize(starts.first.size());

        JointCost_t tenative_gscore = current.gScore;
        std::vector<Neighbor<State, Action, Cost>> joint_neighbor_info =
            cartesian_product.getCurrent();
        assert(joint_neighbor_info.size() == neighbor_joint_state.size());
        for (size_t i = 0; i < neighbor_joint_state.size(); ++i) {
          const auto& e = joint_neighbor_info.at(i);
          neighbor_joint_state.at(i) = e.state;
          neighbor_joint_action.at(i) = e.action;
          neighbor_joint_cost.at(i) = e.cost;
          tenative_gscore.at(i) += e.cost;
        }

        // All variables initialized.
        if (closedSet.find(neighbor_joint_state) != closedSet.end()) {
          continue;
        }

        if (isColliding(neighbor_joint_state, current.state,
                        neighbor_joint_action)) {
          continue;
        }

        //         for (size_t i = 0; i < tenative_gscore.size(); ++i) {
        //           //           std::cout << "Current gScore: " <<
        //           current.gScore.at(i)
        //           //                     << " cost: " <<
        //           joint_neighbor_info.at(i).cost
        //           //                     << " tenative_gscore: " <<
        //           //                     tenative_gscore.at(i)
        //           //                     << " neighbor time: " <<
        //           //                     neighbor_joint_state.at(i).time
        //           //                     << '\n';
        //           assert(tenative_gscore.at(i) ==
        //           neighbor_joint_state.at(i).time);
        //         }

        auto it = stateToHeap.find(neighbor_joint_state);
        if (it == stateToHeap.end()) {
          Cost f_score =
              utils::sum(tenative_gscore) +
              m_env.admissibleJointHeuristic(neighbor_joint_state, goals);
          auto handle = openSet.push(
              Node(neighbor_joint_state, f_score, tenative_gscore));
          (*handle).handle = handle;

          stateToHeap.insert(std::make_pair<>(neighbor_joint_state, handle));
        } else {
          auto handle = it->second;
          // std::cout << "  this is an old node: " << tentative_gScore << ","
          // << (*handle).gScore << std::endl; We found this node before with a
          // better path
          if (tenative_gscore >= (*handle).gScore) {
            continue;
          }

          // update f and gScore
          Cost delta = utils::diffSum((*handle).gScore, tenative_gscore);
          (*handle).gScore = tenative_gscore;
          (*handle).gScoreSum -= delta;
          (*handle).fScore -= delta;
          openSet.increase(handle);
        }

        cameFrom.erase(neighbor_joint_state);
        cameFrom.insert(std::make_pair<>(
            neighbor_joint_state,
            std::make_tuple<>(current.state, neighbor_joint_action,
                              neighbor_joint_cost, tenative_gscore)));

      } while (cartesian_product.increment(), !cartesian_product.atEnd());
    }

    std::cout << "No path found!\n";

    return false;
  }

  bool insertWindowPath(const JointPlan_t& window_path, const Window& window,
                        JointCost_t starts_cost, JointCost_t goals_cost,
                        JointPlan_t& full_path) {
    assert(starts_cost.size() == goals_cost.size());
    assert(window.agent_idxs.size() == goals_cost.size());

    std::cout << "Full:\n";
    for (const auto& i : window.agent_idxs) {
      const auto& e = full_path.at(i);
      for (const auto& p : e.states) {
        std::cout << p.first << ' ' << p.second << "  ";
      }
      std::cout << std::endl;
    }

    for (size_t i = 0; i < window.agent_idxs.size(); ++i) {
      const auto& agent_idx = window.agent_idxs.at(i);
      auto& individual_initial_plan = full_path.at(agent_idx);
      auto& individual_window_plan = window_path.at(i);

      assert(individual_window_plan.states.size() ==
             individual_window_plan.actions.size() + 1);

      const Cost initial_start_idx = starts_cost.at(i);
      const Cost initial_goal_idx = goals_cost.at(i);

      assert(individual_initial_plan.states.at(initial_start_idx).first ==
             individual_window_plan.states.front().first);
      assert(individual_initial_plan.states.at(initial_goal_idx)
                 .first.equalExceptTime(
                     individual_window_plan.states.back().first));

      Cost cost_delta =
          individual_window_plan.states.back().second - initial_goal_idx;

      //       std::cout << "Cost delta: " << cost_delta << '\n';

      for (auto it =
               individual_initial_plan.states.begin() + initial_goal_idx + 1;
           it != individual_initial_plan.states.end(); ++it) {
        it->second += cost_delta;
        it->first.time += cost_delta;
      }

      individual_initial_plan.states.erase(
          individual_initial_plan.states.begin() + initial_start_idx,
          individual_initial_plan.states.begin() + initial_goal_idx + 1);
      individual_initial_plan.states.insert(
          individual_initial_plan.states.begin() + initial_start_idx,
          individual_window_plan.states.begin(),
          individual_window_plan.states.end());

      //       std::cout << "ACTIONS: ";
      //       for (const auto& p : individual_window_plan.actions) {
      //         std::cout << '(' << p.first << ' ' << p.second << ')';
      //       }
      //       std::cout << std::endl;

      individual_initial_plan.actions.erase(
          individual_initial_plan.actions.begin() + initial_start_idx,
          individual_initial_plan.actions.begin() + initial_goal_idx);
      individual_initial_plan.actions.insert(
          individual_initial_plan.actions.begin() + initial_start_idx,
          individual_window_plan.actions.begin(),
          individual_window_plan.actions.end());
    }

    std::cout << "Window:\n";
    for (const auto& e : window_path) {
      for (const auto& p : e.states) {
        std::cout << p.first << ' ' << p.second << "  ";
      }
      //       for (const auto& a : e.actions) {
      //         std::cout << std::endl;
      //       }
      std::cout << std::endl;
    }

    std::cout << "Full:\n";
    for (const auto& i : window.agent_idxs) {
      const auto& e = full_path.at(i);
      for (const auto& p : e.states) {
        std::cout << p.first << ' ' << p.second << "  ";
      }
      std::cout << std::endl;
    }
    //
    //       std::cout << "Starts costs: ";
    //       for (const auto& e : starts_cost) {
    //         std::cout << e << ' ';
    //       }
    //       std::cout << std::endl;
    //
    //       std::cout << "Goals costs: ";
    //       for (const auto& e : goals_cost) {
    //         std::cout << e << ' ';
    //       }
    //       std::cout << std::endl;
    //
    //       //     exit(0);
    //       return true;
    //     }
    return true;
  }

  bool planIndividually(const JointState_t& initial_states,
                        JointPlan_t& solution) {
    solution.resize(initial_states.size());
    for (size_t i = 0; i < initial_states.size(); ++i) {
      LowLevelEnvironment llenv(m_env, i);
      LowLevelSearch_t lowLevel(llenv);
      if (!lowLevel.search(initial_states[i], solution[i])) {
        return false;
      }
    }
    return true;
  }

 public:
  XStar(Environment& environment) : m_env(environment) {}

  bool search(const JointState_t& initial_states, JointPlan_t& solution) {
    if (!planIndividually(initial_states, solution)) return false;

    WPSList_t windows;
    do {
      recWAMPF(windows, solution);
    } while (!shouldQuit());

    return true;
  }
};  // namespace libMultiRobotPlanning

}  // namespace libMultiRobotPlanning
