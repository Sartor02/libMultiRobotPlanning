#pragma once

#include <map>
#include <numeric>

#ifdef USE_FIBONACCI_HEAP
#include <boost/heap/fibonacci_heap.hpp>
#endif
#include <boost/heap/d_ary_heap.hpp>

#include "../example/timer.hpp"
#include "a_star.hpp"
#include "joint_a_star.hpp"

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
          typename Environment,
          typename Window,
          typename StateHasher = std::hash<State>>
class XStar {
 public:
  XStar(Environment& environment) : m_env(environment) {}

  bool search(const std::vector<State>& initialStates,
              std::vector<PlanResult<State, Action, Cost>>& global_solution) {
    Timer t;
    Cost solutionCost = 0;
    global_solution.resize(initialStates.size());

    for (size_t i = 0; i < initialStates.size(); ++i) {
      IndividualEnvironment individualEnvironment(m_env, i);
      IndividualPlanner_t individualPlanner(individualEnvironment);
      bool success =
          individualPlanner.search(initialStates[i], global_solution[i]);
      if (!success) {
        return false;
      }
      solutionCost += global_solution[i].cost;
    }

    Conflict next_conflict;
    std::vector<Window> windows;
    while (m_env.getFirstConflict(global_solution, next_conflict)) {
      const size_t modified_window =
          m_env.AddConflictToWindows(next_conflict, &windows);
      Window& w = windows[modified_window];
      m_env.SetWindowIndices(&w, global_solution);
      std::cout << "Window idx: " << modified_window << '\n';
      std::cout << "Window: " << w << "\n";

      JointState goal(w.agents.size());
      for (size_t i = 0; i < w.agents.size(); ++i) {
        const auto& agent_idx = w.agents[i];
        const auto& goal_idx = w.goal_indices[i];
        goal[i] = (global_solution[agent_idx].states[goal_idx]).first;
      }

      JointEnvironment joint_environment(
          m_env, w.agents, goal, w, global_solution);
      JointAStar<JointState,
                 JointAction,
                 JointCost,
                 TotalCost,
                 JointEnvironment,
                 JointStateHasher>
          joint_a_star(joint_environment);
      JointState start_state(w.agents.size());
      JointAction start_action(w.agents.size());
      JointCost initial_cost(w.agents.size(), 0);
      for (size_t i = 0; i < w.agents.size(); ++i) {
        const auto& agent_idx = w.agents[i];
        const std::pair<State, Cost>& pair =
            global_solution[agent_idx].states[w.start_index];
        start_state[i] = pair.first;
        initial_cost[i] = pair.second;
        start_action[i] = Action::Wait;
      }
      assert(initial_cost.size() == start_state.size());

      m_env.SetWindowIndices(&w, global_solution);

      PlanResult<JointState, JointAction, JointCost> window_solution;
      const bool search_result = joint_a_star.search(
          start_state, start_action, window_solution, initial_cost);
      assert(search_result);
      assert(window_solution.actions.size() == window_solution.states.size());

      for (size_t i = 0; i < w.agents.size(); ++i) {
        std::cout << "Agent: " << w.agents[i] << '\n';
        for (size_t j = 0; j < window_solution.states.size(); ++j) {
          const State& s = window_solution.states[j].first[i];
          std::cout << s << '\n';
        }
      }

      IncorporateToExistingResults(window_solution, w, &global_solution);

      std::cout << "Global solution:\n";
      for (size_t i = 0; i < w.agents.size(); ++i) {
        std::cout << "Agent: " << w.agents[i] << '\n';
        const size_t& agent_idx = w.agents[i];
        const auto& individual_global_plan = global_solution[agent_idx];
        for (const auto& p : individual_global_plan.states) {
          std::cout << p.first << '\n';
        }
      }

      Conflict next_next_conflict;
      if (m_env.getFirstConflict(global_solution, next_next_conflict)) {
        assert(next_next_conflict != next_conflict);
        std::cout << "Current: " << next_conflict << '\n';
        std::cout << "Next: " << next_next_conflict << '\n';
      }
    }

    t.stop();

    std::cout << t.elapsedSeconds() * 1000.0f << '\n';
    return true;
  }

 private:
  struct IndividualEnvironment {
    IndividualEnvironment(Environment& env, size_t agent_index)
        : m_env(env), agent_index(agent_index) {}

    Cost admissibleHeuristic(const State& s) {
      return m_env.admissibleHeuristic(s, agent_index);
    }

    bool isSolution(const State& s) { return m_env.isSolution(s, agent_index); }

    void getNeighbors(const State& s,
                      std::vector<Neighbor<State, Action, Cost>>& neighbors) {
      m_env.getNeighbors(s, agent_index, neighbors);
    }

    void onExpandNode(const State& s, Cost fScore, Cost gScore) {
      m_env.onExpandLowLevelNode(s, fScore, gScore);
    }

    void onDiscover(const State& /*s*/, Cost /*fScore*/, Cost /*gScore*/) {}

   private:
    Environment& m_env;
    size_t agent_index;
  };

  using JointState = std::vector<State>;

  using JointAction = std::vector<Action>;
  using TotalCost = Cost;
  struct JointCost {
    std::vector<Cost> costs;
    JointCost() = default;
    JointCost(const size_t size, const Cost init_value)
        : costs(size, init_value) {}

    JointCost operator+(const JointCost& other) const {
      assert(other.costs.size() == costs.size());
      JointCost result;
      result.costs.resize(other.costs.size());
      for (size_t i = 0; i < costs.size(); ++i) {
        result.costs[i] = costs[i] + other.costs[i];
      }
      return result;
    }

    JointCost operator-(const JointCost& other) const {
      assert(other.costs.size() == costs.size());
      JointCost result;
      result.costs.resize(other.costs.size());
      for (size_t i = 0; i < costs.size(); ++i) {
        result.costs[i] = costs[i] - other.costs[i];
      }
      return result;
    }

    JointCost& operator-=(const JointCost& other) {
      assert(other.costs.size() == costs.size());
      for (size_t i = 0; i < costs.size(); ++i) {
        costs[i] -= other.costs[i];
      }
      return *this;
    }

    Cost& operator[](const size_t index) {
      assert(index < costs.size());
      return costs[index];
    }

    const Cost& operator[](const size_t index) const {
      assert(index < costs.size());
      return costs[index];
    }

    TotalCost GetTotalCost() const {
      TotalCost s = 0;
      for (const auto& c : costs) {
        s += c;
      }
      return s;
    }

    inline size_t size() const { return costs.size(); }
    inline bool empty() const { return costs.empty(); }

    friend std::ostream& operator<<(std::ostream& os, const JointCost& s) {
      for (const auto& c : s.costs) {
        os << c << " ";
      }
      return os;
    }
  };

  std::vector<size_t> GetEndIndices(
      const PlanResult<JointState, JointAction, JointCost>& window_solution,
      const Window& w) {
    std::vector<size_t> end_joint_solution_indices(
        w.agents.size(), window_solution.states.size() - 1);
    const JointCost& max_costs =
        window_solution.states[window_solution.states.size() - 1].second;
    for (size_t plan_step = 0; plan_step < window_solution.states.size();
         ++plan_step) {
      const std::pair<JointState, JointCost>& state_pair =
          window_solution.states[plan_step];
      for (size_t i = 0; i < w.agents.size(); ++i) {
        if (max_costs[i] == state_pair.second[i] &&
            end_joint_solution_indices[i] ==
                window_solution.states.size() - 1) {
          end_joint_solution_indices[i] = plan_step;
        }
      }
    }
    return end_joint_solution_indices;
  }

  void IncorporateToExistingResults(
      const PlanResult<JointState, JointAction, JointCost>& window_solution,
      const Window& w,
      std::vector<PlanResult<State, Action, Cost>>* global_solution) {
    assert(!window_solution.states.empty());
    assert(window_solution.states.size() == window_solution.actions.size());
    assert(w.agents.size() == window_solution.states.at(0).first.size());

    const std::vector<size_t>& end_joint_solution_indices =
        GetEndIndices(window_solution, w);
    for (size_t i = 0; i < w.agents.size(); ++i) {
      std::cout << "Agent: " << w.agents[i]
                << " End cost: " << end_joint_solution_indices[i] + w.start_index << '\n';
    }

    assert(w.goal_indices.size() == w.agents.size());
    for (size_t i = 0; i < w.agents.size(); ++i) {
      PlanResult<State, Action, Cost>& individual_global_plan =
          (*global_solution)[w.agents[i]];
      const size_t& start_idx = w.start_index;
      const size_t& goal_idx = w.goal_indices[i];

      const std::pair<State, Cost> old_goal_state =
          *(individual_global_plan.states.begin() + goal_idx);
      //       const std::pair<Action, Cost> old_action_state =
      //           *(individual_global_plan.actions.begin() + goal_idx);

      const std::pair<State, Cost> new_goal_state = {
          window_solution.states[end_joint_solution_indices[i]].first[i],
          window_solution.states[end_joint_solution_indices[i]].second[i]};
      //       const std::pair<Action, Cost> new_action_state =
      //           {window_solution.actions[end_indices[i]].first[i],
      //           window_solution.actions[end_indices[i]].second[i]};

      const int time_delta =
          new_goal_state.first.time - old_goal_state.first.time;
      const Cost cost_delta = new_goal_state.second - old_goal_state.second;

      individual_global_plan.states.erase(
          individual_global_plan.states.begin() + start_idx,
          individual_global_plan.states.begin() + goal_idx);
      individual_global_plan.actions.erase(
          individual_global_plan.actions.begin() + start_idx,
          individual_global_plan.actions.begin() + goal_idx);
      assert(end_joint_solution_indices[i] > 0);
      for (int window_plan_index = end_joint_solution_indices[i] - 1;
           window_plan_index >= 0;
           --window_plan_index) {
        assert(window_plan_index <
               static_cast<int>(window_solution.states.size()));
        const std::pair<State, Cost>& s = {
            window_solution.states[window_plan_index].first[i],
            window_solution.states[window_plan_index].second[i]};
        assert(window_plan_index <
               static_cast<int>(window_solution.actions.size()));
        const std::pair<Action, Cost>& a = {
            window_solution.actions[window_plan_index].first[i],
            window_solution.actions[window_plan_index].second[i]};
        individual_global_plan.states.insert(
            individual_global_plan.states.begin() + start_idx, s);
        individual_global_plan.actions.insert(
            individual_global_plan.actions.begin() + start_idx, a);
      }

      for (size_t j = end_joint_solution_indices[i] + w.start_index;
           j < individual_global_plan.states.size();
           ++j) {
        individual_global_plan.states[j].first.time += time_delta;
        individual_global_plan.states[j].second += cost_delta;
      }
    }
  }

  struct JointStateHasher {
    size_t operator()(const JointState& js) const {
      size_t seed = 0;
      for (const State& s : js) {
        boost::hash_combine(seed, StateHasher()(s));
      }
      return seed;
    }
  };

  struct JointEnvironment {
    JointEnvironment(
        Environment& env,
        const std::vector<size_t>& agent_indices,
        const JointState& goal,
        const Window& window,
        const std::vector<PlanResult<State, Action, Cost>>& global_solution)
        : m_env(env),
          agent_indices(agent_indices),
          goal(goal),
          window(window),
          global_solution(global_solution) {}

    JointCost AdmissibleJointHeuristic(const JointState& js) {
      JointCost jc(js.size(), 0);
      for (size_t i = 0; i < js.size(); ++i) {
        const State& s = js[i];
        jc[i] = m_env.AdmissibleHeuristic(s, goal[i]);
      }
      assert(js.size() == jc.size());
      return jc;
    }

    bool isSolution(const JointState& js) {
      assert(js.size() == goal.size());
      for (size_t i = 0; i < agent_indices.size(); ++i) {
        if (!js[i].equalExceptTime(goal[i])) {
          return false;
        }
      }
      return true;
    }

    void MakeJointNeighbors(
        const std::vector<std::vector<Neighbor<State, Action, int>>>& v,
        std::vector<Neighbor<JointState, JointAction, JointCost>>* result) {
      using InputT = Neighbor<State, Action, int>;
      auto product = [](long long a, const std::vector<InputT>& b) {
        return a * b.size();
      };
      const long long N = accumulate(v.begin(), v.end(), 1LL, product);
      result->reserve(N);

      std::vector<InputT> u;
      u.reserve(v.size());

      for (long long n = 0; n < N; ++n) {
        lldiv_t q{n, 0};
        for (long long i = v.size() - 1; 0 <= i; --i) {
          q = div(q.quot, v[i].size());
          u[i] = v[i][q.rem];
        }

        Neighbor<JointState, JointAction, JointCost> neighbor(
            JointState(v.size()),
            JointAction(v.size()),
            JointCost(v.size(), 0));

        assert(!neighbor.state.empty());
        assert(!neighbor.action.empty());
        assert(!neighbor.cost.costs.empty());

        // Normal iteration does not work due to use of reserve(); size() is
        // wrong.
        for (size_t i = 0; i < v.size(); ++i) {
          const auto& x = u[i];
          neighbor.state[i] = (x.state);
          neighbor.action[i] = (x.action);
          neighbor.cost.costs[i] = (x.cost);
        }

        assert(!neighbor.state.empty());
        assert(!neighbor.action.empty());
        assert(!neighbor.cost.costs.empty());

        result->emplace_back(neighbor);
      }
    }

    void GetJointNeighbors(
        const JointState& js,
        std::vector<Neighbor<JointState, JointAction, JointCost>>* neighbors) {
      assert(js.size() == agent_indices.size());
      neighbors->clear();
      std::vector<std::vector<Neighbor<State, Action, int>>>
          individual_neighbors(js.size());
      //       std::cout << "Joint State: ";
      //       for (const auto& s : js) {
      //         std::cout << s << " ";
      //       }
      //       std::cout << '\n';
      //       std::cout << "Individual Neighbors:\n";

      for (size_t i = 0; i < js.size(); ++i) {
        const size_t& agent_idx = agent_indices[i];
        const State& s = js[i];

        // Follow existing path into window.
        if (!window.IsInWindow(s.x, s.y) &&
            (s.time <
             static_cast<int>(global_solution[agent_idx].states.size() - 1)) &&
            (s == global_solution[agent_idx].states[s.time].first)) {
          const State& next_state =
              global_solution[agent_idx].states[s.time + 1].first;
          const Action next_action =
              global_solution[agent_idx].actions[s.time].first;
          const Cost next_cost =
              global_solution[agent_idx].actions[s.time].second;
          individual_neighbors[i] = {
              Neighbor<State, Action, int>(next_state, next_action, next_cost)};
          //           std::cout << "Follow path: " << s << "\n";
        } else {
          //           if (!window.IsInWindow(s.x, s.y)) {
          //             std::cout << "Not in window but not on path: " << s <<
          //             " Size check: " <<
          //             static_cast<int>(global_solution[agent_idx].states.size()
          //             - 1) << " Access: " <<
          //             (global_solution[agent_idx].states[s.time].first) <<
          //             "\n";
          //           }

          if (s.equalExceptTime(goal[i])) {
            individual_neighbors[i].emplace_back(Neighbor<State, Action, int>(
                {s.time + 1, s.x, s.y}, Action::Wait, 0));
            //             std::cout << "At goal: " << s << "\n";
          }

          m_env.getNeighbors(s, agent_idx, individual_neighbors[i]);
          //           std::cout << "True Neighbors!\n";
        }
        //         for (const auto& n : individual_neighbors[i]) {
        //           std::cout << n.state << " ";
        //         }
        //         std::cout << '\n';
      }

      //       for (const size_t& agent_idx : agent_indices) {
      //         const PlanResult<State, Action, Cost>& individual_global_plan =
      //             global_solution[agent_idx];
      //         std::cout << "Agent Idx: " << agent_idx << '\n';
      //         for (const auto& p : individual_global_plan.states) {
      //           std::cout << p.first << '\n';
      //         }
      //       }

      MakeJointNeighbors(individual_neighbors, neighbors);
    }

    bool IsValidNeighbor(const JointState& old_js, const JointState& new_js) {
      assert(old_js.size() == new_js.size());
      for (size_t i = 0; i < new_js.size(); ++i) {
        //         std::cout << "Old: " << old_js[i].time << " New: " <<
        //         new_js[i].time << '\n';
        assert(old_js[i].time + 1 == new_js[i].time ||
               old_js[i].time == new_js[i].time);
      }
      for (size_t i = 0; i < new_js.size(); ++i) {
        if (!window.IsInWindow(new_js[i].x, new_js[i].y)) {
          const size_t& agent_idx = agent_indices[i];
          const auto& states = global_solution[agent_idx].states;
          const size_t& global_solution_idx = new_js[i].time;
          if (states.size() > global_solution_idx &&
              !(states[global_solution_idx].first == new_js[i])) {
            //             std::cout << "Not in window: " << new_js[i] << "\n";
            return false;
          }
        }
        for (size_t j = i + 1; j < new_js.size(); ++j) {
          if (new_js[i] == (new_js[j])) {
            //             std::cout << "Collide with self\n";
            return false;
          }

          if ((old_js[i].equalExceptTime(new_js[j])) &&
              (old_js[i].time + 1 == new_js[j].time) &&
              (new_js[i].equalExceptTime(old_js[j])) &&
              (new_js[i].time == old_js[j].time + 1)) {
            //             std::cout << "Collide swap\n";
            return false;
          }
        }
      }
      return true;
    }

    void onExpandNode(const JointState& /*s*/,
                      TotalCost /*fScore*/,
                      TotalCost /*gScore*/) {
      //       m_env.onExpandLowLevelNode(s, fScore, gScore);
    }

    void onDiscover(const JointState& /*s*/,
                    JointCost /*fScore*/,
                    JointCost /*gScore*/) {}

    //    private:
    Environment& m_env;
    const std::vector<size_t>& agent_indices;
    const JointState& goal;
    const Window& window;
    const std::vector<PlanResult<State, Action, Cost>>& global_solution;
  };

 private:
  Environment& m_env;
  using IndividualPlanner_t = AStar<State, Action, Cost, IndividualEnvironment>;
};

}  // namespace libMultiRobotPlanning
