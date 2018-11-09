#pragma once

#include <map>

#ifdef USE_FIBONACCI_HEAP
#include <boost/heap/fibonacci_heap.hpp>
#endif
#include <boost/heap/d_ary_heap.hpp>

#include "a_star.hpp"

namespace libMultiRobotPlanning {

/*!
  \example cbs.cpp Example that solves the Multi-Agent Path-Finding (MAPF)
  problem in a 2D grid world with up/down/left/right
  actions
*/

/*! \brief

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
template <typename State,
          typename Action,
          typename Cost,
          typename Conflict,
          typename Constraints,
          typename Environment,
          typename Window>
class XStar {
 public:
  XStar(Environment& environment) : m_env(environment) {}

  bool search(const std::vector<State>& initialStates,
              std::vector<PlanResult<State, Action, Cost>>& solution) {
    Cost solutionCost = 0;
    std::vector<Constraints> constraints;
    constraints.resize(initialStates.size());
    solution.resize(initialStates.size());

    for (size_t i = 0; i < initialStates.size(); ++i) {
      IndividualEnvironment individualEnvironment(m_env, i, constraints[i]);
      IndividualPlanner_t individualPlanner(individualEnvironment);
      bool success = individualPlanner.search(initialStates[i], solution[i]);
      if (!success) {
        return false;
      }
      solutionCost += solution[i].cost;
    }

    std::vector<Conflict> conflicts = m_env.getAllConflicts(solution);
    if (conflicts.empty()) {
      std::cout << "done; cost: " << solutionCost << std::endl;
      return true;
    }

    std::vector<Window> windows = m_env.createWindowsFromConflicts(conflicts);
    for (auto& w : windows) {
      m_env.setWindowIndices(w, solution);
      searchInWindow(w, solution);
      std::cout << "Window: " << w << "\n";
    }

    std::cout << "Repair not finished!\n";
    return false;
  }

 private:
  struct IndividualEnvironment {
    IndividualEnvironment(Environment& env,
                          size_t agentIdx,
                          const Constraints& constraints)
        : m_env(env) {
      m_env.setLowLevelContext(agentIdx, &constraints);
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
      m_env.onExpandLowLevelNode(s, fScore, gScore);
    }

    void onDiscover(const State& /*s*/, Cost /*fScore*/, Cost /*gScore*/) {}

   private:
    Environment& m_env;
  };

  std::vector<PlanResult<State, Action, Cost>> searchInWindow(
      const Window& w,
      const std::vector<PlanResult<State, Action, Cost>>& current_solution) {
    assert(!w.agents.empty());
    std::vector<PlanResult<State, Action, Cost>> solutions;
    solutions.resize(w.agents.size());

    OpenSet_t openSet;
    StateToHeapMap_t stateToHeap;
    ClosedSet_t closedSet;
    CameFromMap_t cameFrom;

    JointState start_state;
    const Cost start_cost =
        current_solution[w.agents[0]].states[w.start_index].second;
    for (const size_t& ai : w.agents) {
      assert(start_cost == current_solution[ai].states[w.start_index].second);
      start_state.push_back(current_solution[ai].states[w.start_index].first);
    }

    std::cout << "Start cost: " << start_cost << " Start state: ";
    for (const auto& s : start_state) {
      std::cout << s << " ";
    }
    std::cout << '\n';

    return solutions;
  }

 private:
  using JointState = std::vector<State>;
  using JointAction = std::vector<std::tuple<Action, Cost, Cost>>;
  using JointCost = std::vector<Cost>;

  struct JointStateHasher {
    size_t operator()(const JointState& js) const {
      size_t seed = 0;
      for (const auto& s : js) {
        boost::hash_combine(seed, std::hash<State>(s));
      }
      return seed;
    }
  };

  struct JointNode {
    JointNode(const JointState& state,
              const JointCost& fScore,
              const JointCost& gScore)
        : state(state),
          fScore(fScore),
          gScore(gScore),
          totalFScore(0),
          totalGScore(0) {
      for (const Cost& c : fScore) {
        totalFScore += c;
      }

      for (const Cost& c : gScore) {
        totalGScore += c;
      }

      Validate();
    }

    void Validate() const {
      assert(state.size() == fScore.size());
      assert(state.size() == gScore.size());
    }

    bool operator<(const JointNode& other) const {
      Validate();
      other.Validate();

      // Sort order
      // 1. lowest fScore
      // 2. highest gScore

      // Our heap is a maximum heap, so we invert the comperator function here
      if (totalFScore != other.fScore) {
        return totalFScore > other.fScore;
      } else {
        return totalGScore < other.gScore;
      }
    }

    friend std::ostream& operator<<(std::ostream& os, const JointNode& node) {
      os << "state: ";
      for (const auto& s : node.state) {
        os << s << " ";
      }
      os << " fScore: " << node.totalFScore << " gScore: " << node.totalGScore;
      return os;
    }

    JointState state;
    JointCost fScore;
    JointCost gScore;
    Cost totalFScore;
    Cost totalGScore;

#ifdef USE_FIBONACCI_HEAP
    typename boost::heap::fibonacci_heap<Node>::handle_type handle;
#else
    typename boost::heap::d_ary_heap<JointNode,
                                     boost::heap::arity<2>,
                                     boost::heap::mutable_<true>>::handle_type
        handle;
#endif
  };

#ifdef USE_FIBONACCI_HEAP
  using OpenSet_t = typename boost::heap::fibonacci_heap<JointNode>;
  using HeapHandle_t = typename OpenSet_t::handle_type;
#else
  using OpenSet_t = typename boost::heap::
      d_ary_heap<JointNode, boost::heap::arity<2>, boost::heap::mutable_<true>>;
  using HeapHandle_t = typename OpenSet_t::handle_type;
#endif

  using StateToHeapMap_t =
      std::unordered_map<JointState, HeapHandle_t, JointStateHasher>;
  using ClosedSet_t = std::unordered_set<JointState, JointStateHasher>;
  using CameFromMap_t =
      std::unordered_map<JointState,
                         std::tuple<JointState, JointAction, Cost>,
                         JointStateHasher>;

 private:
  Environment& m_env;
  using IndividualPlanner_t = AStar<State, Action, Cost, IndividualEnvironment>;
};

}  // namespace libMultiRobotPlanning
