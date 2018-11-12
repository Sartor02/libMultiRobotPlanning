#pragma once

#ifdef USE_FIBONACCI_HEAP
#include <boost/heap/fibonacci_heap.hpp>
#endif

#include <boost/heap/d_ary_heap.hpp>
#include <unordered_map>
#include <unordered_set>

#include "neighbor.hpp"
#include "planresult.hpp"

namespace libMultiRobotPlanning {

/*!
  \example a_star.cpp Simple example using a 2D grid world and
  up/down/left/right
  actions
*/

/*! \brief A* Algorithm to find the shortest path

This class implements the A* algorithm. A* is an informed search algorithm
that finds the shortest path for a given map. It can use a heuristic that
needsto be admissible.

This class can either use a fibonacci heap, or a d-ary heap. The latter is the
default. Define "USE_FIBONACCI_HEAP" to use the fibonacci heap instead.

\tparam State Custom state for the search. Needs to be copy'able
\tparam Action Custom action for the search. Needs to be copy'able
\tparam Cost Custom Cost type (integer or floating point types)
\tparam Environment This class needs to provide the custom A* logic. In
    particular, it needs to support the following functions:
  - `Cost admissibleHeuristic(const State& s)`\n
    This function can return 0 if no suitable heuristic is available.

  - `bool isSolution(const State& s)`\n
    Return true if the given state is a goal state.

  - `void getNeighbors(const State& s, std::vector<Neighbor<State, Action,
   int> >& neighbors)`\n
    Fill the list of neighboring state for the given state s.

  - `void onExpandNode(const State& s, int fScore, int gScore)`\n
    This function is called on every expansion and can be used for statistical
purposes.

  - `void onDiscover(const State& s, int fScore, int gScore)`\n
    This function is called on every node discovery and can be used for
   statistical purposes.

    \tparam StateHasher A class to convert a state to a hash value. Default:
   std::hash<State>
*/
template <typename JointState,
          typename JointAction,
          typename JointCost,
          typename TotalCost,
          typename Environment,
          typename StateHasher = std::hash<JointState>>
class JointAStar {
 public:
  JointAStar(Environment& environment) : m_env(environment) {}

  bool search(const JointState& start_state,
              const JointAction& start_action,
              PlanResult<JointState, JointAction, JointCost>& solution,
              JointCost initial_cost) {
    solution.states.clear();
    solution.states.push_back(std::make_pair<>(start_state, initial_cost));
    solution.actions.clear();
    solution.cost = initial_cost;

    std::cout << "Joint A* Start state: ";
    for (const auto& s : start_state) {
      std::cout << s << ' ';
    }
    std::cout << '\n';

    std::cout << "Joint A* Goal state: ";
    for (const auto& s : m_env.goal) {
      std::cout << s << ' ';
    }
    std::cout << '\n';
    
    for (size_t i = 0; i < m_env.goal.size(); ++i) {
      for (size_t j = i + 1; j < m_env.goal.size(); ++j) {
        assert(!m_env.goal[i].equalExceptTime(m_env.goal[j]));
      }
    }

    openSet_t openSet;
    stateMap_t stateToHeap;
    closedSet_t closedSet;
    cameFromMap_t cameFrom;

    auto handle = openSet.push(
        Node(start_state,
             m_env.AdmissibleJointHeuristic(start_state) + initial_cost,
             initial_cost));
    stateToHeap.insert(std::make_pair<>(start_state, handle));
    (*handle).handle = handle;

    std::vector<Neighbor<JointState, JointAction, JointCost>> neighbors;
    neighbors.reserve(10);

    JointState prev_state = start_state;
    for (auto& e : prev_state) {
      e.time = -100;
    }

    while (!openSet.empty()) {
      Node current = openSet.top();

//                   std::cout << "Current state:";
//                   for (const auto& s : current.state) {
//                     std::cout << s << ' ';
//                   }
//                   std::cout << '\n';

      for (auto& e : prev_state) {
        e.time += 1;
      }
      if (prev_state == current.state) {
        std::cerr << "Loop detected!\n";
        return false;
      }
      prev_state = current.state;

      m_env.onExpandNode(
          current.state, current.totalFScore, current.totalGScore);

      if (m_env.isSolution(current.state)) {
        solution.states.clear();
        solution.actions.clear();
        auto iter = cameFrom.find(current.state);
        while (iter != cameFrom.end()) {
          const JointState& key = iter->first;
          const std::tuple<JointState, JointAction, JointCost, TotalCost>&
              value = iter->second;

          solution.states.push_back(std::make_pair<>(key, std::get<2>(value)));
          solution.actions.push_back(
              std::make_pair<>(std::get<1>(value), std::get<2>(value)));
          iter = cameFrom.find(std::get<0>(value));
        }
        solution.states.push_back(std::make_pair<>(start_state, initial_cost));
        solution.actions.push_back(
            std::make_pair<>(start_action, initial_cost));
        std::reverse(solution.states.begin(), solution.states.end());
        std::reverse(solution.actions.begin(), solution.actions.end());
        solution.cost = current.gScore;
        solution.fmin = current.fScore;

        assert(solution.states.size() == solution.actions.size());

        return true;
      }

      openSet.pop();
      stateToHeap.erase(current.state);
      closedSet.insert(current.state);

      // traverse neighbors
      neighbors.clear();
      m_env.GetJointNeighbors(current.state, &neighbors);
      for (const Neighbor<JointState, JointAction, JointCost>& neighbor :
           neighbors) {
        assert(!neighbor.state.empty());
        assert(!neighbor.action.empty());
        assert(!neighbor.cost.empty());
        assert(neighbor.state.size() == neighbor.cost.size());
        assert(neighbor.action.size() == neighbor.cost.size());
        assert(current.state.size() == neighbor.cost.size());

        //         std::cout << "Neighbor state:";
        //         for (const auto& s : neighbor.state) {
        //           std::cout << s << ' ';
        //         }
        //         std::cout << '\n';

        if (closedSet.find(neighbor.state) != closedSet.end()) {
          //           std::cout << "Rejected by closed set\n";
          continue;
        }

        if (!m_env.IsValidNeighbor(current.state, neighbor.state)) {
          //           std::cout << "Neighbor reject\n";
          continue;
        }
        //         std::cout << "Neighbor kept\n";

        const JointCost tentative_gScore = current.gScore + neighbor.cost;

        assert(current.state.size() == tentative_gScore.size());
        auto iter = stateToHeap.find(neighbor.state);
        if (iter == stateToHeap.end()) {  // Discover a new node
          const JointCost fScore =
              tentative_gScore + m_env.AdmissibleJointHeuristic(neighbor.state);
          assert(current.state.size() == fScore.size());
          auto handle =
              openSet.push(Node(neighbor.state, fScore, tentative_gScore));
          (*handle).handle = handle;
          stateToHeap.insert(std::make_pair<>(neighbor.state, handle));
          m_env.onDiscover(neighbor.state, fScore, tentative_gScore);
          // std::cout << "  this is a new node " << fScore << "," <<
          // tentative_gScore << std::endl;
        } else {
          auto handle = iter->second;
          // std::cout << "  this is an old node: " << tentative_gScore << ","
          // << (*handle).gScore << std::endl;
          // We found this node before with a better path
          if (tentative_gScore.GetTotalCost() >=
              (*handle).gScore.GetTotalCost()) {
            continue;
          }

          // update f and gScore
          const JointCost delta = (*handle).gScore - tentative_gScore;
          assert(current.state.size() == delta.size());
          (*handle).gScore = tentative_gScore;
          (*handle).fScore -= delta;
          (*handle).totalGScore = (*handle).gScore.GetTotalCost();
          (*handle).totalGScore = (*handle).fScore.GetTotalCost();
          assert(current.state.size() == (*handle).gScore.size());
          assert(current.state.size() == (*handle).fScore.size());
          openSet.increase(handle);
          m_env.onDiscover(neighbor.state, (*handle).fScore, (*handle).gScore);
        }

        // Best path for this node so far
        // TODO: this is not the best way to update "cameFrom", but otherwise
        // default c'tors of State and Action are required
        cameFrom.erase(neighbor.state);
        cameFrom.insert(std::make_pair<>(
            neighbor.state,
            std::make_tuple<>(current.state,
                              neighbor.action,
                              tentative_gScore,
                              tentative_gScore.GetTotalCost())));
      }
    }

    return false;
  }

 private:
  struct Node {
    Node(const JointState& state, JointCost fScore, JointCost gScore)
        : state(state),
          fScore(fScore),
          gScore(gScore),
          totalFScore(0),
          totalGScore(0) {
      for (const auto& c : fScore.costs) {
        totalFScore += c;
      }
      for (const auto& c : gScore.costs) {
        totalGScore += c;
      }
    }

    bool operator<(const Node& other) const {
      // Sort order
      // 1. lowest fScore
      // 2. highest gScore

      // Our heap is a maximum heap, so we invert the comperator function here
      if (totalFScore != other.totalFScore) {
        return totalFScore > other.totalFScore;
      } else {
        return totalGScore < other.totalGScore;
      }
    }

    friend std::ostream& operator<<(std::ostream& os, const Node& node) {
      os << "state: " << node.state << " fScore: " << node.fScore
         << " gScore: " << node.gScore;
      return os;
    }

    JointState state;
    JointCost fScore;
    JointCost gScore;
    TotalCost totalFScore;
    TotalCost totalGScore;

#ifdef USE_FIBONACCI_HEAP
    typename boost::heap::fibonacci_heap<Node>::handle_type handle;
#else
    typename boost::heap::d_ary_heap<Node,
                                     boost::heap::arity<2>,
                                     boost::heap::mutable_<true>>::handle_type
        handle;
#endif
  };

#ifdef USE_FIBONACCI_HEAP
  using openSet_t = typename boost::heap::fibonacci_heap<Node> openSet_t;
  using fibHeapHandle_t = typename openSet_t::handle_type;
#else
  using openSet_t = typename boost::heap::
      d_ary_heap<Node, boost::heap::arity<2>, boost::heap::mutable_<true>>;
  using fibHeapHandle_t = typename openSet_t::handle_type;
#endif

  using stateMap_t =
      std::unordered_map<JointState, fibHeapHandle_t, StateHasher>;
  using closedSet_t = std::unordered_set<JointState, StateHasher>;
  using cameFromMap_t =
      std::unordered_map<JointState,
                         std::tuple<JointState,
                                    JointAction,
                                    JointCost /* Neighbor Cost*/,
                                    TotalCost /* Tenative G Score */>,
                         StateHasher>;

 private:
  Environment& m_env;
};

}  // namespace libMultiRobotPlanning
