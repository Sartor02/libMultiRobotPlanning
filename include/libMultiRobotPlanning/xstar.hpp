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
#include "../example/timer.hpp"
#include "utils.hpp"

namespace libMultiRobotPlanning {

static constexpr bool kDebug = false;

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

  static void print(const JointState_t& ss, std::ostream& os = std::cout) {
    for (const auto& s : ss) {
      os << s << ' ';
    }
  }

  struct Node {
    Node(const JointState_t& state, const JointAction_t& action,
         const Cost& fScore, const JointCost_t& gScore)
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

  using open_set_t =
      typename boost::heap::d_ary_heap<Node, boost::heap::arity<2>,
                                       boost::heap::mutable_<true>>;
  using open_set_handle_t = typename open_set_t::handle_type;

  using state_to_heap_t =
      std::unordered_map<JointState_t, open_set_handle_t, StateHasher>;

  using closed_set_t =
      std::unordered_map<JointState_t, JointCost_t, StateHasher>;

  using came_from_value_t =
      std::tuple<JointState_t, JointAction_t, JointCost_t, JointCost_t>;
  using came_from_t =
      std::unordered_map<JointState_t, came_from_value_t, StateHasher>;

  using out_of_window_t = std::unordered_set<JointState_t, StateHasher>;

  struct SearchState {
    static constexpr Cost kDefaultCost = -1;
    bool disabled;
    Cost min_cost;
    Cost max_cost;
    open_set_t open_set;
    state_to_heap_t state_to_heap;
    closed_set_t closed_set;
    came_from_t came_from;
    out_of_window_t out_of_window;

    SearchState()
        : disabled(false),
          min_cost(kDefaultCost),
          max_cost(kDefaultCost),
          open_set(),
          state_to_heap(),
          closed_set(),
          came_from() {}

    bool operator==(const SearchState& o) const {
      return disabled == o.disabled && min_cost == o.min_cost &&
             max_cost == o.max_cost && open_set == o.open_set &&
             state_to_heap == o.state_to_heap && closed_set == o.closed_set &&
             came_from == o.came_from && out_of_window == o.out_of_window;
    }

    bool wasSearchRestricted() const {
      if (disabled) {
        return false;
      }
      if (min_cost == kDefaultCost || max_cost == kDefaultCost) {
        return true;
      }
      return (!out_of_window.empty());
    }

    friend std::ostream& operator<<(std::ostream& os, const SearchState& ps) {
      os << "Enabled: " << (ps.enabled ? "true" : "false");
      return os;
    }
  };

  using SSListIndex_t = size_t;
  using SSList_t = utils::StableStorage<SearchState, 1000>;

  struct WindowPlannerState {
    Window window;
    SSListIndex_t ss_index;
    SSList_t* ss_list;

    WindowPlannerState() = delete;
    WindowPlannerState(const Window& window, SSList_t* ss_list)
        : window(window), ss_index(ss_list->add()), ss_list(ss_list) {}

    bool operator==(const WindowPlannerState& o) const {
      return (window == o.window) && (ss_index == o.ss_index) &&
             (ss_list == o.ss_list);
    }

    WindowPlannerState merge(const WindowPlannerState& o) const {
      return {window.merge(o.window), ss_list};
    }

    bool overlapping(const WindowPlannerState& other) const {
      // If no overlap in space, then they cannot be overlapping in paths.
      if (!window.intersects(other.window)) {
        return false;
      }

      const SearchState* ss = getSearchState();
      const SearchState* oss = other.getSearchState();
      assert(ss->min_cost != SearchState::kDefaultCost);
      assert(ss->max_cost != SearchState::kDefaultCost);
      assert(oss->min_cost != SearchState::kDefaultCost);
      assert(oss->max_cost != SearchState::kDefaultCost);

      bool oss_min_inside =
          ss->min_cost <= oss->min_cost && ss->max_cost >= oss->min_cost;
      bool oss_max_inside =
          ss->min_cost <= oss->max_cost && ss->max_cost >= oss->max_cost;
      bool ss_min_inside =
          ss->min_cost >= oss->min_cost && ss->min_cost <= oss->max_cost;
      bool ss_max_inside =
          ss->max_cost >= oss->min_cost && ss->max_cost <= oss->max_cost;

      if (!(oss_min_inside || oss_max_inside || ss_min_inside ||
            ss_max_inside)) {
        return false;
      }

      // If they overlap in space and share agents, then they need to be merged.
      if (window.has_overlapping_agents(other.window)) {
        return true;
      }

      return false;
    }

    SearchState* getSearchState() const { return &(ss_list->at(ss_index)); }

    friend std::ostream& operator<<(std::ostream& os,
                                    const WindowPlannerState& ws) {
      return os << "Window: " << ws.window
                << " Search state index: " << ws.ss_index;
    }
  };

  using WPS_t = WindowPlannerState;
  using WPSList_t = std::vector<WindowPlannerState>;

  void growAndMergeExisting(WPSList_t* windows, JointPlan_t* solution) {
    for (size_t i = 0; i < windows->size(); ++i) {
      WPS_t& wi = windows->at(i);
      growAndReplanIn(&wi, solution);

      bool found_overlapping = false;
      do {
        found_overlapping = false;
        for (size_t j = 0; j < windows->size(); ++j) {
          if (i == j) {
            continue;
          }
          const WPS_t& wj = windows->at(j);
          if (wi.overlapping(wj)) {
            found_overlapping = true;
            if (kDebug) {
              std::cout << "Merging two existing windows " << wi << ", " << wj
                        << std::endl;
            }
            wi.getSearchState()->disabled = true;
            wj.getSearchState()->disabled = true;
            wi = wi.merge(wj);
            while (!planIn(&wi, solution)) {
              wi.window.grow();
            }

            // Erase wj.
            windows->erase(windows->begin() + j);

            // If j < i, then erasing j will cause i to be shifted back by 1, so
            // the reference and the index need to be updated.
            if (j < i) {
              --i;
              wi = windows->at(i);
            }

            break;
          }
        }
      } while (found_overlapping);
    }
  }

  void integrateNewConflictWindow(WPS_t window, WPSList_t* windows,
                                  JointPlan_t* solution) {
    while (!planIn(&window, solution)) {
      window.window.grow();
    }
    bool found_overlapping = false;
    do {
      found_overlapping = false;
      for (size_t i = 0; i < windows->size(); ++i) {
        const WPS_t& wi = windows->at(i);
        if (window.overlapping(wi)) {
          found_overlapping = true;
          window.getSearchState()->disabled = true;
          wi.getSearchState()->disabled = true;
          window = window.merge(wi);
          while (!planIn(&window, solution)) {
            window.window.grow();
          }
          windows->erase(windows->begin() + i);
          break;
        }
      }
    } while (found_overlapping);
    windows->push_back(window);
  }

  void removeCompletedWindows(WPSList_t* windows) {
    for (size_t i = 0; i < windows->size();) {
      auto& w = (*windows)[i];
      if (!w.getSearchState()->wasSearchRestricted()) {
        windows->erase(windows->begin() + i);
      } else {
        ++i;
      }
    }
  }

  bool recWAMPF(WPSList_t* windows, SSList_t* search_states,
                JointPlan_t* solution) {
    if (kDebug) {
      std::cout << "Starting recWAMPF\n";
    }
    growAndMergeExisting(windows, solution);

    Conflict result;
    while (m_env.getFirstConflict(*solution, result)) {
      if (kDebug) {
        std::cout << "Found first conflict\n";
        std::cout << "Conflict: " << result << std::endl;
      }
      WPS_t window(m_env.createWindowFromConflict(result), search_states);
      integrateNewConflictWindow(window, windows, solution);
    }

    removeCompletedWindows(windows);
    if (kDebug) {
      std::cout << "Finished all conflicts\n";
    }
    return true;
  }

  bool existsOverlapping(const WPS_t& given_w, const WPSList_t& list) const {
    for (size_t i = 0; i < list.size(); ++i) {
      const WPS_t& w = list[i];
      if (given_w.overlapping(w)) {
        return true;
      }
    }
    return false;
  }
  
  void AStarSearchUntil() {}

  void Stage1(WPS_t* window, const JointState_t& goals) {
    SearchState* ss = window->getSearchState();
    for (const JointState_t& s : ss->out_of_window) {
      auto it = ss->came_from.find(s);
      assert(it != ss->came_from.end());
      const JointAction_t& a = std::get<1>(it->second);
      const JointCost_t& g_score = std::get<3>(it->second);
      insertStateIntoOpen(goals, s, a, g_score, &(ss->state_to_heap),
                          &(ss->open_set));
    }
    ss->out_of_window.clear();
    
    AStarSearchUntil();
  }

  std::pair<JointState_t, JointAction_t> getStateAction(
      const size_t idx, const JointPlan_t& plan) {
    JointState_t js(plan.size());
    JointAction_t ja(plan.size());

    for (size_t i = 0; i < plan.size(); ++i) {
      assert(plan[i].states.size() == plan[i].actions.size());
      js[i] = plan[i].states.at(idx).first;
      ja[i] = plan[i].actions.at(idx).first;
    }

    return {js, ja};
  }

  struct NodeAndCameFromValues_t {
    JointCost_t cost_between_starts;
    std::vector<Node> nodes;
    std::vector<std::pair<JointState_t, came_from_value_t>> came_from_values;

    void verify() {
      assert(nodes.size() == (came_from_values.size() + 1));
      assert(!cost_between_starts.empty());
    }
  };

  void Stage2(WPS_t* window,
              const NodeAndCameFromValues_t& info_between_starts) {
    SearchState* ss = window->getSearchState();
    const auto& cost_between_starts = info_between_starts.cost_between_starts;
    
    // Update openlist.
    for (auto& kv : ss->state_to_heap) {
      auto& handle = kv.second;
      assert((*handle).g_score.size() == cost_between_starts.size());
      for (size_t i = 0; i < cost_between_starts.size(); ++i) {
        (*handle).g_score[i] += cost_between_starts[i];
        (*handle).g_score_sum += cost_between_starts[i];
        (*handle).f_score += cost_between_starts[i];
      }
      ss->open_set.decrease(handle);
    }

    // Update closedlist.
    for (auto& kv : ss->closed_set) {
      JointCost_t& g_value = kv.second;
      assert(g_value.size() == cost_between_starts.size());
      for (size_t i = 0; i < cost_between_starts.size(); ++i) {
        g_value[i] += cost_between_starts[i];
      }
    }

    // Insert path into openlist.
    for (size_t i = 0; i < info_between_starts.came_from_values.size(); ++i) {
      const Node& node = info_between_starts.nodes[i];
      const std::pair<JointState_t, came_from_value_t>& came_from_info = info_between_starts.came_from_values[i];
      auto handle =
          ss->open_set.push(node);
      (*handle).handle = handle;
      ss->state_to_heap[node.state] = handle;
      ss->came_from[came_from_info.first] = came_from_info.second;
    }
    
    AStarSearchUntil();
  }

  void verifyStartCostDifference(const JointCost_t& before_starts_costs,
                                 const JointCost_t& after_starts_costs) {
    assert(before_starts_costs.size() == after_starts_costs.size());
    assert(!before_starts_costs.empty());
    const Cost delta = after_starts_costs.front() - before_starts_costs.front();
    for (size_t i = 0; i < before_starts_costs.size(); ++i) {
      assert(delta == (after_starts_costs.at(i) - before_starts_costs.at(i)));
    }
  }

  std::tuple<JointPlan_t, JointCost_t> trimPlanBetweenCosts(
      const WPS_t& window, const JointPlan_t& solution,
      const JointState_t& before_starts, const JointCost_t& before_starts_costs,
      const JointState_t& after_starts, const JointCost_t& after_starts_costs) {
    verifyStartCostDifference(before_starts_costs, after_starts_costs);
    JointPlan_t between_starts_solution;

    for (const size_t& i : window.window.agent_idxs) {
      between_starts_solution.push_back(solution.at(i));
    }
    assert(between_starts_solution.size() == before_starts_costs.size());
    assert(before_starts.size() == before_starts_costs.size());
    assert(before_starts.size() == after_starts.size());

    JointCost_t between_starts_cost = before_starts_costs;

    assert(between_starts_solution.size() == after_starts_costs.size());
    for (size_t i = 0; i < between_starts_solution.size(); ++i) {
      IndividualPlan_t& p = between_starts_solution.at(i);
      const Cost min_cost = after_starts_costs.at(i);
      const Cost max_cost = before_starts_costs.at(i);
      between_starts_cost.at(i) = max_cost - min_cost;

      if (kDebug) {
        std::cout << "Before trim: ";
        for (const auto& e : p.states) {
          std::cout << e.second << ' ';
        }
        std::cout << std::endl;

        std::cout << "Before trim: ";
        for (const auto& a : p.actions) {
          std::cout << a.first << ' ';
        }
        std::cout << std::endl;

        std::cout << "min cost: " << min_cost << std::endl;
        std::cout << "max cost: " << max_cost << std::endl;
      }

      p.states.erase(p.states.begin() + max_cost + 1, p.states.end());
      p.states.erase(p.states.begin(), p.states.begin() + min_cost);

      p.actions.erase(p.actions.begin() + max_cost + 1, p.actions.end());
      p.actions.erase(p.actions.begin(), p.actions.begin() + min_cost);

      if (kDebug) {
        std::cout << "After trim: ";
        for (const auto& e : p.states) {
          std::cout << e.second << ' ';
        }
        std::cout << std::endl;

        std::cout << "After trim: ";
        for (const auto& a : p.actions) {
          std::cout << a.first << ' ';
        }
        std::cout << std::endl;

        std::cout << p.states.back().second << " vs " << max_cost << std::endl;
      }
      assert(p.states.back().second == max_cost);
    }

    for (size_t i = 0; i < between_starts_solution.size(); ++i) {
      assert(between_starts_solution.at(i).states.front().first ==
             after_starts.at(i));
      assert(between_starts_solution.at(i).states.back().first ==
             before_starts.at(i));
    }

    return {between_starts_solution, between_starts_cost};
  }

  std::pair<JointState_t, JointCost_t> getIthState(
      const JointPlan_t& joint_plan, const size_t i) {
    JointState_t s;
    JointCost_t c;
    for (const auto& p : joint_plan) {
      assert(i < p.states.size());
      s.push_back(p.states[i].first);
      c.push_back(p.states[i].second);
    }
    return {s, c};
  }

  std::pair<JointAction_t, JointCost_t> getIthAction(
      const JointPlan_t& joint_plan, const size_t i) {
    JointAction_t a;
    JointCost_t c;
    for (const auto& p : joint_plan) {
      assert(i < p.actions.size());
      a.push_back(p.actions[i].first);
      c.push_back(p.actions[i].second);
    }
    return {a, c};
  }

  NodeAndCameFromValues_t extractPathBetweenStarts(
      const WPS_t& window, const JointPlan_t& solution,
      const JointState_t& before_starts, const JointCost_t& before_starts_costs,
      const JointState_t& after_starts, const JointCost_t& after_starts_costs) {
    JointPlan_t between_starts_solution;
    NodeAndCameFromValues_t nodes_and_came_from;
    std::tie(between_starts_solution, nodes_and_came_from.cost_between_starts) =
        trimPlanBetweenCosts(window, solution, before_starts,
                             before_starts_costs, after_starts,
                             after_starts_costs);

    const size_t num_steps = between_starts_solution.front().states.size();

    for (size_t step = 1; step < num_steps; ++step) {
      const std::pair<JointState_t, JointCost_t> current_state =
          getIthState(solution, step);
      const std::pair<JointAction_t, JointCost_t> current_action =
          getIthAction(solution, step);

      const std::pair<JointState_t, JointCost_t> previous_state =
          getIthState(solution, step - 1);
      const std::pair<JointAction_t, JointCost_t> previous_action =
          getIthAction(solution, step - 1);

      nodes_and_came_from.nodes.push_back(
          Node(current_state.first, current_action.first,
               utils::sum(current_state.second), current_state.second));

      nodes_and_came_from.came_from_values.push_back(
          {current_state.first,
           came_from_value_t(previous_state.first, current_action.first,
                             current_action.second, current_state.second)});
    }

    const std::pair<JointState_t, JointCost_t> current_state =
        getIthState(solution, 0);
    const std::pair<JointAction_t, JointCost_t> current_action =
        getIthAction(solution, 0);
    nodes_and_came_from.nodes.push_back(
        Node(current_state.first, current_action.first,
             utils::sum(current_state.second), current_state.second));

    nodes_and_came_from.verify();
    return nodes_and_came_from;
  }

  void growAndReplanIn(WPS_t* window, JointPlan_t* solution) {
    if (kDebug) {
      std::cout << "Grow and replan in" << std::endl;
    }

    JointState_t before_starts;
    JointCost_t before_starts_costs;
    JointState_t before_goals;
    JointCost_t before_goals_costs;
    std::tie(before_starts, before_starts_costs, before_goals,
             before_goals_costs) = window->window.getStartsAndGoals(*solution);

    window->window.grow();

    JointState_t after_starts;
    JointCost_t after_starts_costs;
    JointState_t after_goals;
    JointCost_t after_goals_costs;
    std::tie(after_starts, after_starts_costs, after_goals, after_goals_costs) =
        getCollisionFreeStartsGoals(*solution, window);

    verifyStartCostDifference(before_starts_costs, after_starts_costs);
    const NodeAndCameFromValues_t info_between_starts =
        extractPathBetweenStarts(*window, *solution, before_starts,
                                 before_starts_costs, after_starts,
                                 after_starts_costs);

    Stage1(window, after_goals);
    Stage2(window, info_between_starts);

    while (!planIn(window, solution)) {
      window->window.grow();
    }
  }

  bool shouldQuit(const WPSList_t& windows) {
    static int iter = 0;
    std::cout
        << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> ITERATION:"
        << iter++ << std::endl;
    if (iter >= 2) return true;

    return windows.empty();
  }

  bool isColliding(const JointState_t& n, const JointState_t& c) {
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

  bool positionsCollide(const JointState_t& positions) const {
    for (size_t i = 0; i < positions.size(); ++i) {
      for (size_t j = i + 1; j < positions.size(); ++j) {
        if (positions[i].equalExceptTime(positions[j])) {
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
    assert(goal.size() == start_cost.size());
    assert(goal.size() == goal_cost.size());
    for (size_t i = 0; i < start.size(); ++i) {
      assert(start.at(i).time == start.front().time);
      assert(start_cost.at(i) == start_cost.front());
      assert(start_cost.at(i) <= goal_cost.at(i));
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

    assert(!positionsCollide(start));
    assert(!positionsCollide(goal));
  }

  std::tuple<JointState_t, JointCost_t, JointState_t, JointCost_t>
  getCollisionFreeStartsGoals(const JointPlan_t& solution, WPS_t* window) {
    JointState_t sg_starts;
    JointCost_t sg_starts_costs;
    JointState_t sg_goals;
    JointCost_t sg_goals_costs;
    std::tie(sg_starts, sg_starts_costs, sg_goals, sg_goals_costs) =
        window->window.getStartsAndGoals(solution);
    while (positionsCollide(sg_goals)) {
      if (kDebug) {
        std::cout << "goals collide, expanding" << std::endl;
      }
      window->window.grow();
      std::tie(sg_starts, sg_starts_costs, sg_goals, sg_goals_costs) =
          window->window.getStartsAndGoals(solution);
    }
    if (kDebug) {
      verifyPlanInEndpoints(sg_starts, sg_starts_costs, sg_goals,
                            sg_goals_costs);
    }

    return {sg_starts, sg_starts_costs, sg_goals, sg_goals_costs};
  }

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

  bool insertStateIntoOpen(const JointState_t& goals,
                           const JointState_t& neighbor_joint_state,
                           const JointAction_t& neighbor_joint_action,
                           const JointCost_t& neighbor_tenative_gscore,
                           state_to_heap_t* state_to_heap,
                           open_set_t* open_set) {
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
      if (utils::sum(neighbor_tenative_gscore) >=
          utils::sum((*handle).g_score)) {
        return true;
      }
      // Update f and g score.
      Cost delta = utils::diffSum((*handle).g_score, neighbor_tenative_gscore);
      assert(delta > 0);
      (*handle).g_score = neighbor_tenative_gscore;
      (*handle).g_score_sum -= delta;
      (*handle).f_score -= delta;
      open_set->increase(handle);
    }

    return false;
  }

  void processNeighbor(const JointState_t& starts, const JointState_t& goals,
                       const Node& current,
                       const JointNeighbor_t& joint_neighbor_info,
                       const bool& is_in_window, const closed_set_t& closed_set,
                       state_to_heap_t* state_to_heap, open_set_t* open_set,
                       came_from_t* came_from, out_of_window_t* out_of_window) {
    JointState_t neighbor_joint_state(starts.size());
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

    if (isColliding(neighbor_joint_state, current.state)) {
      return;
    }

    if (!is_in_window) {
      auto it = came_from->find(neighbor_joint_state);
      if (it != came_from->end()) {
        assert(out_of_window->find(neighbor_joint_state) !=
               out_of_window->end());
        if (utils::sum(std::get<3>(it->second)) <=
            utils::sum(neighbor_tenative_gscore)) {
          return;
        }
      } else {
        assert(out_of_window->find(neighbor_joint_state) ==
               out_of_window->end());
      }
      (*came_from)[neighbor_joint_state] =
          std::make_tuple<>(current.state, neighbor_joint_action,
                            neighbor_joint_cost, neighbor_tenative_gscore);
      out_of_window->insert(neighbor_joint_state);
      return;
    }

    if (closed_set.find(neighbor_joint_state) != closed_set.end()) {
      return;
    }

    if (insertStateIntoOpen(goals, neighbor_joint_state, neighbor_joint_action,
                            neighbor_tenative_gscore, state_to_heap,
                            open_set)) {
      return;
    }

    (*came_from)[neighbor_joint_state] =
        std::make_tuple<>(current.state, neighbor_joint_action,
                          neighbor_joint_cost, neighbor_tenative_gscore);
  }

  bool isTooManyIterations(const size_t& iterations, const WPS_t& window) {
    return (iterations > std::pow(100, window.window.agent_idxs.size()));
  }

  std::pair<Cost, Cost> getMinMaxCostFromPlan(const JointPlan_t& joint_plan) {
    Cost min_cost = std::numeric_limits<Cost>::max();
    Cost max_cost = std::numeric_limits<Cost>::min();
    for (const IndividualPlan_t& individual_plan : joint_plan) {
      min_cost = std::min(individual_plan.states.front().second, min_cost);
      max_cost = std::max(individual_plan.states.back().second, max_cost);
    }

    assert(min_cost != std::numeric_limits<Cost>::max());
    assert(max_cost != std::numeric_limits<Cost>::min());
    return {min_cost, max_cost};
  }

  void extractJointNeighborInfo(
      JointNeighbor_t* joint_neighbor_info, bool* joint_neighbor_in_window,
      const std::vector<std::pair<Neighbor<State, Action, int>, bool>>&
          generator_output) {
    (*joint_neighbor_in_window) = true;
    for (size_t i = 0; i < generator_output.size(); ++i) {
      (*joint_neighbor_info)[i] = std::move(generator_output[i].first);
      (*joint_neighbor_in_window) =
          (*joint_neighbor_in_window) && generator_output[i].second;
    }
  }

  bool planIn(WPS_t* window, JointPlan_t* solution) {
    assert(!(window->window.agent_idxs.empty()));

    JointState_t starts;
    JointCost_t starts_costs;
    JointState_t goals;
    JointCost_t goals_costs;
    std::tie(starts, starts_costs, goals, goals_costs) =
        getCollisionFreeStartsGoals(*solution, window);
    if (kDebug) {
      std::cout << "Plan in: " << window->window << '\n';
    }

    auto* ss = window->getSearchState();
    open_set_t& open_set = ss->open_set;
    state_to_heap_t& state_to_heap = ss->state_to_heap;
    closed_set_t& closed_set = ss->closed_set;
    came_from_t& came_from = ss->came_from;
    out_of_window_t& out_of_window = ss->out_of_window;

    open_set.clear();
    state_to_heap.clear();
    closed_set.clear();
    came_from.clear();
    out_of_window.clear();

    auto handle = open_set.push(
        Node(starts, JointAction_t(starts.size(), Action::None),
             m_env.admissibleJointHeuristic(starts, goals), starts_costs));
    state_to_heap.insert(std::make_pair<>(starts, handle));
    (*handle).handle = handle;

    for (size_t expand_count = 0; !open_set.empty(); ++expand_count) {
      if (isTooManyIterations(expand_count, *window)) {
        return false;
      }

      Node current = open_set.top();

      m_env.onExpandNode(current.state, current.f_score, current.g_score);

      if (m_env.isJointSolution(current.state, goals)) {
        const auto window_solution =
            unwindPath(starts, starts_costs, current, came_from);
        const auto min_max = getMinMaxCostFromPlan(window_solution);
        for (const auto& e : starts_costs) {
          assert(min_max.first == e);
        }
        ss->min_cost = min_max.first;
        ss->max_cost = min_max.second;
        return insertWindowPath(window_solution, window->window, starts_costs,
                                goals_costs, solution);
      }

      open_set.pop();
      state_to_heap.erase(current.state);
      closed_set.insert({current.state, current.g_score});

      auto neighbor_generator = m_env.getJointWindowNeighbors(
          current.state, current.action, goals, window->window, *solution);

      JointNeighbor_t joint_neighbor_info(current.state.size());
      bool joint_neighbor_in_window = true;
      do {
        const auto& generator_output = neighbor_generator.getAndIncrement();
        extractJointNeighborInfo(&joint_neighbor_info,
                                 &joint_neighbor_in_window, generator_output);
        processNeighbor(starts, goals, current, joint_neighbor_info,
                        joint_neighbor_in_window, closed_set, &state_to_heap,
                        &open_set, &came_from, &out_of_window);

      } while (!neighbor_generator.atEnd());
    }
    return false;
  }

  bool insertWindowPath(const JointPlan_t& window_path, const Window& window,
                        const JointCost_t& starts_cost,
                        const JointCost_t& goals_cost, JointPlan_t* full_path) {
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
      auto& individual_initial_plan = full_path->at(agent_idx);
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
    Timer individual_timer;
    if (!planIndividually(initial_states, solution)) return false;
    individual_timer.stop();

    std::cout << "Initial time: " << individual_timer.elapsedSeconds()
              << std::endl;

    auto time_so_far = individual_timer.elapsedSeconds();

    WPSList_t windows;
    SSList_t search_states;
    do {
      Timer timer;
      recWAMPF(&windows, &search_states, &solution);
      timer.stop();
      time_so_far += timer.elapsedSeconds();
      std::cout << "Time so far: " << time_so_far << std::endl;
    } while (!shouldQuit(windows));

    return true;
  }
};  // namespace libMultiRobotPlanning

}  // namespace libMultiRobotPlanning
