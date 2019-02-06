#pragma once

#include <map>

#include "a_star.hpp"

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
          typename Window, typename Environment>
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
                      std::vector<Neighbor<State, Action, Cost> >& neighbors) {
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

  struct SearchState {
    friend std::ostream& operator<<(std::ostream& os, const SearchState& ps) {
      return os;
    }
  };

  Environment& m_env;

  using JointState_t = std::vector<State>;
  using LowLevelSearch_t = AStar<State, Action, Cost, LowLevelEnvironment>;
  using IndividualPlan_t = PlanResult<State, Action, Cost>;
  using JointPlan_t = std::vector<IndividualPlan_t>;

  struct WindowPlannerState {
    Window window;
    SearchState search_state;

    WindowPlannerState() = delete;
    explicit WindowPlannerState(const Window& window)
        : window(window), search_state() {}
    WindowPlannerState(const Window& window, const SearchState* search_state)
        : window(window), search_state(search_state) {}

    WindowPlannerState merge(const WindowPlannerState& o) const {
      return {window.merge(o.window)};
    }

    bool overlapping(const WindowPlannerState& other) const { return false; }

    friend std::ostream& operator<<(std::ostream& os,
                                    const WindowPlannerState& ws) {
      return os << "Window: " << ws.window
                << " Search state: " << ws.search_state;
    }
  };

  using WPS_t = WindowPlannerState;
  using WPSList_t = std::vector<WindowPlannerState>;

  bool recWAMPF(WPSList_t& windows, JointPlan_t& solution) {
    for (WPS_t& window : windows) {
      growAndReplanIn(window, solution);
      // if overlapping
      //    planInOverlapWindows
    }

    Conflict result;
    while (m_env.getFirstConflict(solution, result)) {
      WPS_t w =
          WindowPlannerState(m_env.createWindowFromConflict(result, solution));
      // planInOverlapWindows
    }
    return true;
  }

  void growAndReplanIn(WPS_t& window, JointPlan_t& solution) {}

  bool shouldQuit() { return true; }

  bool windowOverlapsWithOther(const WPS_t& window,
                               const WPSList_t& windows) const {
    return true;
  }

  bool planInOverlapWindows(WPS_t& window, WPSList_t& windows) { return true; }

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
};

}  // namespace libMultiRobotPlanning
