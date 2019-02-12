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
  using JointNeighbor_t = std::vector<Neighbor<State, Action, Cost>>;

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
      const WPS_t& w = windows[i];
      if (&given_window == &w) {
        // Ignore self.
        continue;
      }

      if (given_window.overlapping(w)) {
        std::cout << "Merging with window " << i << '\n';
        given_window = given_window.merge(w);
        windows.erase(windows.begin() + i);
        while (!planIn(given_window, solution)) {
          given_window.window.grow();
        }
        planned_in = true;
      } else {
        ++i;
      }
    }

    if (!planned_in) {
      while (!planIn(given_window, solution)) {
        given_window.window.grow();
      }
    }
    std::cout << "Windows push back\n";
    windows.push_back(given_window);
    return true;
  }

  struct Node {
    Node(const JointState_t& state, const JointAction_t& action, Cost fScore,
         JointCost_t gScore)
        : state(state),
          action(action),
          f_score(fScore),
          g_score_sum(utils::sum(gScore)),
          g_score(gScore) {}

    bool operator<(const Node& other) const {
      // Sort order
      // 1. lowest fScore
      // 2. highest gScore

      // Our heap is a maximum heap, so we invert the comperator function here
      if (f_score != other.f_score) {
        return f_score > other.f_score;
      } else {
        return g_score_sum < other.g_score_sum;
      }
    }

    friend std::ostream& operator<<(std::ostream& os, const Node& node) {
      os << "state: " << node.state << " fScore: " << node.f_score
         << " gScore: " << node.g_score_sum;
      return os;
    }

    JointState_t state;
    JointAction_t action;

    Cost f_score;
    Cost g_score_sum;
    JointCost_t g_score;

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

    assert(start.size() == goal.size());

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

  bool goalsCollide(const JointState_t& goals) {
    for (size_t i = 0; i < goals.size(); ++i) {
      for (size_t j = i + 1; j < goals.size(); ++j) {
        if (goals[i].equalExceptTime(goals[j])) {
          return true;
        }
      }
    }
    return false;
  }

  void getCollisionFreeStartsGoals(const JointPlan_t& solution, WPS_t* window,
                                   JointState_t* starts,
                                   JointCost_t* starts_costs,
                                   JointState_t* goals,
                                   JointCost_t* goals_costs) {
    auto starts_goals = window->window.getStartsAndGoals(solution);
    while (goalsCollide(starts_goals.second.first)) {
      window->window.grow();
      starts_goals = window->window.getStartsAndGoals(solution);
    }
    *starts = starts_goals.first.first;
    *starts_costs = starts_goals.first.second;
    *goals = starts_goals.second.first;
    *goals_costs = starts_goals.second.second;
    verifyPlanInEndpoints(*starts, *starts_costs, *goals, *goals_costs);
  }

  using open_set_t =
      typename boost::heap::d_ary_heap<Node, boost::heap::arity<2>,
                                       boost::heap::mutable_<true>>;
  using open_set_handle_t = typename open_set_t::handle_type;

  using state_to_heap_t =
      std::unordered_map<JointState_t, open_set_handle_t, StateHasher>;

  using closed_set_t = std::unordered_set<JointState_t, StateHasher>;

  using came_from_t = std::unordered_map<
      JointState_t,
      std::tuple<JointState_t, JointAction_t, JointCost_t, JointCost_t>,
      StateHasher>;

  JointPlan_t unwindPath(const JointState_t& starts,
                         const JointCost_t& starts_costs,
                         const Node& unwind_start_node,
                         const came_from_t& came_from) {
    JointPlan_t window_solution(starts.size());
    auto iter = came_from.find(unwind_start_node.state);
    while (iter != came_from.end()) {
      for (size_t i = 0; i < iter->first.size(); ++i) {
        auto& i_solution = window_solution.at(i);
        const State& s_i = iter->first.at(i);
        const Action& a_i = std::get<1>(iter->second).at(i);
        const Cost& c_i = std::get<2>(iter->second).at(i);
        const Cost& g_i = std::get<3>(iter->second).at(i);

        i_solution.states.push_back({s_i, g_i});
        i_solution.actions.push_back({a_i, c_i});
      }
      iter = came_from.find(std::get<0>(iter->second));
    }
    for (size_t i = 0; i < window_solution.size(); ++i) {
      auto& solution = window_solution[i];
      solution.states.push_back(
          std::make_pair<>(starts.at(i), starts_costs.at(i)));
      std::reverse(solution.states.begin(), solution.states.end());
      std::reverse(solution.actions.begin(), solution.actions.end());
      solution.cost = utils::sum(unwind_start_node.g_score);
      solution.fmin = unwind_start_node.f_score;
    }
    return window_solution;
  }

  void processNeighbor(const JointState_t& starts, const JointState_t& goals,
                       const Node& current,
                       const JointNeighbor_t& joint_neighbor_info,
                       const closed_set_t& closed_set,
                       state_to_heap_t* state_to_heap, open_set_t* open_set,
                       came_from_t* came_from) {
    JointState_t neighbor_joint_state = starts;
    JointAction_t neighbor_joint_action(starts.size());
    JointCost_t neighbor_joint_cost(starts.size());
    JointCost_t neighbor_tenative_gscore = current.g_score;
    assert(joint_neighbor_info.size() == neighbor_joint_state.size());
    for (size_t i = 0; i < neighbor_joint_state.size(); ++i) {
      const auto& e = joint_neighbor_info[i];
      neighbor_joint_state[i] = e.state;
      neighbor_joint_action[i] = e.action;
      neighbor_joint_cost[i] = e.cost;
      neighbor_tenative_gscore[i] += e.cost;
    }

    // All variables initialized.
    if (closed_set.find(neighbor_joint_state) != closed_set.end()) {
      return;
    }

    if (isColliding(neighbor_joint_state, current.state,
                    neighbor_joint_action)) {
      return;
    }

    auto it = state_to_heap->find(neighbor_joint_state);
    if (it == state_to_heap->end()) {
      Cost f_score =
          utils::sum(neighbor_tenative_gscore) +
          m_env.admissibleJointHeuristic(neighbor_joint_state, goals);
      auto handle =
          open_set->push(Node(neighbor_joint_state, neighbor_joint_action,
                              f_score, neighbor_tenative_gscore));
      (*handle).handle = handle;
      state_to_heap->insert(std::make_pair<>(neighbor_joint_state, handle));
    } else {
      auto handle = it->second;
      if (neighbor_tenative_gscore >= (*handle).g_score) {
        return;
      }
      // Update f and g score.
      Cost delta = utils::diffSum((*handle).g_score, neighbor_tenative_gscore);
      (*handle).g_score = neighbor_tenative_gscore;
      (*handle).g_score_sum -= delta;
      (*handle).f_score -= delta;
      open_set->increase(handle);
    }

    came_from->erase(neighbor_joint_state);
    came_from->insert(std::make_pair<>(
        neighbor_joint_state,
        std::make_tuple<>(current.state, neighbor_joint_action,
                          neighbor_joint_cost, neighbor_tenative_gscore)));
  }
  
  bool isTooManyIterations(const size_t& iterations, const WPS_t& window) {
    return (iterations > std::pow(100, window.window.agent_idxs.size()));
  }

  bool planIn(WPS_t& window, JointPlan_t& solution) {
    std::cout << "Plan in: " << window.window << '\n';

    JointState_t starts;
    JointCost_t starts_costs;
    JointState_t goals;
    JointCost_t goals_costs;
    getCollisionFreeStartsGoals(solution, &window, &starts, &starts_costs,
                                &goals, &goals_costs);

    open_set_t open_set;
    state_to_heap_t state_to_heap;
    closed_set_t closed_set;
    came_from_t came_from;

    auto handle = open_set.push(
        Node(starts, JointAction_t(starts.size(), Action::None),
             m_env.admissibleJointHeuristic(starts, goals), starts_costs));
    state_to_heap.insert(std::make_pair<>(starts, handle));
    (*handle).handle = handle;

    for (size_t expand_count = 0; !open_set.empty(); ++expand_count) {
      if (isTooManyIterations(expand_count, window)) {
        return false;
      }
      Node current = open_set.top();
      m_env.onExpandNode(current.state, current.f_score, current.g_score);

      if (m_env.isJointSolution(current.state, goals)) {
        const auto window_solution =
            unwindPath(starts, starts_costs, current, came_from);
        return insertWindowPath(window_solution, window.window, starts_costs,
                                goals_costs, solution);
      }

      open_set.pop();
      state_to_heap.erase(current.state);
      closed_set.insert(current.state);

      auto neighbor_generator = m_env.getInWindowJointWindowNeighbors(
          current.state, current.action, goals, window.window, solution);
      do {
        const JointNeighbor_t& joint_neighbor_info =
            neighbor_generator.getCurrent();
        processNeighbor(starts, goals, current, joint_neighbor_info, closed_set,
                        &state_to_heap, &open_set, &came_from);

      } while (neighbor_generator.increment(), !neighbor_generator.atEnd());
    }
    return false;
  }

  bool insertWindowPath(const JointPlan_t& window_path, const Window& window,
                        JointCost_t starts_cost, JointCost_t goals_cost,
                        JointPlan_t& full_path) {
    assert(starts_cost.size() == goals_cost.size());
    assert(window.agent_idxs.size() == goals_cost.size());

    //     std::cout << "Full:\n";
    //     for (const auto& i : window.agent_idxs) {
    //       const auto& e = full_path.at(i);
    //       for (const auto& p : e.states) {
    //         std::cout << p.first << ' ' << p.second << "  ";
    //       }
    //       std::cout << std::endl;
    //     }

    //     std::cout << "Window:\n";
    //     for (const auto& e : window_path) {
    //       for (const auto& p : e.states) {
    //         std::cout << p.first << ' ' << p.second << "  ";
    //       }
    //       std::cout << std::endl;
    //       for (const auto& a : e.actions) {
    //         std::cout << a.first << " " << a.second << " ";
    //       }
    //       std::cout << std::endl;
    //     }

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

      size_t ignore_wait_command_count = 0;
      for (auto it = individual_window_plan.actions.rbegin();
           it != individual_window_plan.actions.rend(); ++it) {
        if (it->second == 0 || it->first == Action::GoalWait) {
          ignore_wait_command_count++;
        } else {
          break;
        }
      }
      assert(ignore_wait_command_count < individual_window_plan.actions.size());
      //       std::cout << "Ignoring last " << ignore_wait_command_count
      //                 << " commands\n";

      individual_initial_plan.states.erase(
          individual_initial_plan.states.begin() + initial_start_idx,
          individual_initial_plan.states.begin() + initial_goal_idx + 1);
      individual_initial_plan.states.insert(
          individual_initial_plan.states.begin() + initial_start_idx,
          individual_window_plan.states.begin(),
          individual_window_plan.states.end() - ignore_wait_command_count);

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
          individual_window_plan.actions.end() - ignore_wait_command_count);
    }

    //     std::cout << "Full:\n";
    //     for (const auto& i : window.agent_idxs) {
    //       const auto& e = full_path.at(i);
    //       for (const auto& p : e.states) {
    //         std::cout << p.first << ' ' << p.second << "  ";
    //       }
    //       std::cout << std::endl;
    //     }
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
