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
#include "../example/fake_timer.hpp"
#include "../example/timer.hpp"
#include "utils.hpp"

namespace libMultiRobotPlanning {

static constexpr bool kTiming = true;
static constexpr bool kProduction = true;

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
  template <class Timer>
  struct TimingASUExp_t {
    Timer total_ASUExp;
    Timer time_top_o;
    Timer time_check_in_c;
    Timer time_add_parent;
    size_t num_neighbors = 0;
    Timer time_add_neighbors_to_o;
  };
  using TimingASUExp = TimingASUExp_t<Timer>;

  template <class Timer>
  struct TimingAStarSearchUntil_t {
    Timer total_astar_search_until;
    size_t num_values_f_less_fmax = 0;
    TimingASUExp timing_ASUExp;
  };
  using TimingAStarSearchUntil = TimingAStarSearchUntil_t<Timer>;

  template <class Timer>
  struct TimingStage1_t {
    Timer total_Stage1;
    Timer time_add_x_to_o;
    Timer time_clear_x;
    TimingAStarSearchUntil timing_AStarSearchUntil;
  };
  using TimingStage1 = TimingStage1_t<Timer>;

  template <class Timer>
  struct TimingStage2_t {
    Timer total_Stage2;
    size_t num_len_path = 0;
    Timer time_expanding_s;
    TimingAStarSearchUntil timing_AStarSearchUntil;
  };
  using TimingStage2 = TimingStage2_t<Timer>;

  template <class Timer>
  struct TimingS3Exp_t {
    Timer total_S3Exp;
    Timer time_top_o;
    Timer time_check_in_c;
    Timer time_add_parent;
    size_t num_neighbors = 0;
    Timer time_add_neighbors_to_o;
  };
  using TimingS3Exp = TimingS3Exp_t<Timer>;

  template <class Timer>
  struct TimingStage3_t {
    Timer total_Stage3;
    size_t num_until_goal_expanded = 0;
    TimingS3Exp timing_S3Exp;
  };
  using TimingStage3 = TimingStage3_t<Timer>;

  template <class Timer>
  struct TimingGARI_t {
    Timer total_GARI;
    Timer time_path_btw_starts;
    TimingStage1 timing_stage1;
    TimingStage2 timing_stage2;
    TimingStage3 timing_stage3;
  };
  using TimingGARI = TimingGARI_t<Timer>;

  template <class Timer>
  struct TimingPlanIn_t {
    Timer total_PlanIn;
    size_t num_until_goal_expanded = 0;
    TimingS3Exp timing_expansions;
  };
  using TimingPlanIn = TimingPlanIn_t<Timer>;

  template <class Timer>
  struct TimingPIOW_t {
    Timer total_PIOW;
    size_t num_windows = 0;
    Timer time_check_overlap;
    size_t num_overlapping_windows = 0;
    Timer time_merge_windows;
    Timer time_remove_window;
    Timer time_add_to_windows;
    TimingPlanIn timing_PlanIn;
  };
  using TimingPIOW = TimingPIOW_t<Timer>;

  template <class Timer>
  struct TimingRecWAMPF_t {
    Timer total_RecWAMPF;
    size_t num_windows = 0;
    size_t num_max_agents_in_window = 0;
    TimingGARI timing_gari;
    TimingPIOW timing_PIOW_overlapping;
    size_t num_new_collisions = 0;
    TimingPIOW timing_PIOW_new_collisions;
    size_t num_windows_should_quit = 0;
    Timer time_should_quit;
    Timer time_remove_window;
  };
  using TimingRecWAMPF = TimingRecWAMPF_t<Timer>;
  
  template <class Timer>
  struct TimingWAMPF_t {
    Timer total_WAMPF;
    bool is_optimal = false;
    float optimality_bound = 0;
    std::vector<float> successive_bounds;
    std::vector<float> successive_runtimes;
    size_t num_agents = 0;
    Timer time_individual_plan;
    Timer time_first_plan;
    size_t num_max_agents_in_window_first_iteration = 0;
    size_t num_recWAMPF = 0;
    TimingRecWAMPF timing_recWAMPF;
    // clang-format off
    friend std::ostream& operator<<(std::ostream& os, const TimingWAMPF_t& t) {
      os << "Total time: " << t.total_WAMPF << "\n"
      "is_optimal: " << (t.is_optimal ? "true" : "false") << "\n"
      "optimality_bound: " << t.optimality_bound << "\n"
      "successive_bounds: "; utils::list_to_string(os, t.successive_bounds); os << "\n"
      "successive_runtimes: "; utils::list_to_string(os, t.successive_runtimes); os << "\n"
      "num_agents: " << t.num_agents << "\n"
      "time_individual_plan: " << t.time_individual_plan << "\n"
      "time_first_plan: " << t.time_first_plan << "\n"
      "num_max_agents_in_window_first_iteration: " << t.num_max_agents_in_window_first_iteration << "\n"
      "num_recWAMPF: " << t.num_recWAMPF<< "\n"
      "timing_recWAMPF:\n"
      "    total_RecWAMPF: " << t.timing_recWAMPF.total_RecWAMPF<< "\n"
      "    num_windows: " << t.timing_recWAMPF.num_windows<< "\n"
      "    num_max_agents_in_window: " << t.timing_recWAMPF.num_max_agents_in_window << "\n"
      "    timing_gari:\n"
      "        total_GARI: " << t.timing_recWAMPF.timing_gari.total_GARI<< "\n"
      "        time_path_btw_starts: " << t.timing_recWAMPF.timing_gari.time_path_btw_starts<< "\n"
      "        Stage1:\n"
      "            total_Stage1:" << t.timing_recWAMPF.timing_gari.timing_stage1.total_Stage1<< "\n"
      "            time_add_x_to_o: " << t.timing_recWAMPF.timing_gari.timing_stage1.time_add_x_to_o<< "\n"
      "            time_clear_x: " << t.timing_recWAMPF.timing_gari.timing_stage1.time_clear_x<< "\n"
      "            timing_AStarSearchUntil:\n"
      "                total_astar_search_until:" << t.timing_recWAMPF.timing_gari.timing_stage1.timing_AStarSearchUntil.total_astar_search_until<< "\n"
      "                num_values_f_less_fmax:" << t.timing_recWAMPF.timing_gari.timing_stage1.timing_AStarSearchUntil.num_values_f_less_fmax<< "\n"
      "                ASU Exp:\n"
      "                    total_ASUExp:" << t.timing_recWAMPF.timing_gari.timing_stage1.timing_AStarSearchUntil.timing_ASUExp.total_ASUExp << "\n"
      "                    time_top_o:" << t.timing_recWAMPF.timing_gari.timing_stage1.timing_AStarSearchUntil.timing_ASUExp.time_top_o<< "\n"
      "                    time_check_in_c:" << t.timing_recWAMPF.timing_gari.timing_stage1.timing_AStarSearchUntil.timing_ASUExp.time_check_in_c << "\n"
      "                    time_add_parent:" << t.timing_recWAMPF.timing_gari.timing_stage1.timing_AStarSearchUntil.timing_ASUExp.time_add_parent<< "\n"
      "                    num_neighbors:" << t.timing_recWAMPF.timing_gari.timing_stage1.timing_AStarSearchUntil.timing_ASUExp.num_neighbors << "\n"
      "                    time_add_neighbors_to_o:" << t.timing_recWAMPF.timing_gari.timing_stage1.timing_AStarSearchUntil.timing_ASUExp.time_add_neighbors_to_o << "\n"
      "        Stage2:\n"
      "            total_Stage2:" << t.timing_recWAMPF.timing_gari.timing_stage2.total_Stage2<< "\n"
      "            num_len_path:" << t.timing_recWAMPF.timing_gari.timing_stage2.num_len_path<< "\n"
      "            time_expanding_s:" << t.timing_recWAMPF.timing_gari.timing_stage2.time_expanding_s<< "\n"
      "            timing_AStarSearchUntil:\n"
      "                total_astar_search_until:" << t.timing_recWAMPF.timing_gari.timing_stage2.timing_AStarSearchUntil.total_astar_search_until<< "\n"
      "                num_values_f_less_fmax:" << t.timing_recWAMPF.timing_gari.timing_stage2.timing_AStarSearchUntil.num_values_f_less_fmax<< "\n"
      "                ASU Exp:\n"
      "                    total_ASUExp:" << t.timing_recWAMPF.timing_gari.timing_stage2.timing_AStarSearchUntil.timing_ASUExp.total_ASUExp << "\n"
      "                    time_top_o:" << t.timing_recWAMPF.timing_gari.timing_stage2.timing_AStarSearchUntil.timing_ASUExp.time_top_o<< "\n"
      "                    time_check_in_c:" << t.timing_recWAMPF.timing_gari.timing_stage2.timing_AStarSearchUntil.timing_ASUExp.time_check_in_c << "\n"
      "                    time_add_parent:" << t.timing_recWAMPF.timing_gari.timing_stage2.timing_AStarSearchUntil.timing_ASUExp.time_add_parent<< "\n"
      "                    num_neighbors:" << t.timing_recWAMPF.timing_gari.timing_stage2.timing_AStarSearchUntil.timing_ASUExp.num_neighbors << "\n"
      "                    time_add_neighbors_to_o:" << t.timing_recWAMPF.timing_gari.timing_stage2.timing_AStarSearchUntil.timing_ASUExp.time_add_neighbors_to_o << "\n"
      "        Stage3:\n"
      "            total_Stage3:" << t.timing_recWAMPF.timing_gari.timing_stage3.total_Stage3<< "\n"
      "            num_until_goal_expanded:" << t.timing_recWAMPF.timing_gari.timing_stage3.num_until_goal_expanded<< "\n"
      "            timing_S3Exp:\n"
      "                total_S3Exp: " << t.timing_recWAMPF.timing_gari.timing_stage3.timing_S3Exp.total_S3Exp<< "\n"
      "                time_top_o: " << t.timing_recWAMPF.timing_gari.timing_stage3.timing_S3Exp.time_top_o<< "\n"
      "                time_check_in_c: " << t.timing_recWAMPF.timing_gari.timing_stage3.timing_S3Exp.time_check_in_c<< "\n"
      "                time_add_parent: " << t.timing_recWAMPF.timing_gari.timing_stage3.timing_S3Exp.time_add_parent<< "\n"
      "                num_neighbors: " << t.timing_recWAMPF.timing_gari.timing_stage3.timing_S3Exp.num_neighbors<< "\n"
      "                time_add_neighbors_to_o: " << t.timing_recWAMPF.timing_gari.timing_stage3.timing_S3Exp.time_add_neighbors_to_o<< "\n"
      "    timing_PIOW_overlapping:\n"
      "        total_PIOW: " << t.timing_recWAMPF.timing_PIOW_overlapping.total_PIOW<< "\n"
      "        num_windows: " << t.timing_recWAMPF.timing_PIOW_overlapping.num_windows<< "\n"
      "        time_check_overlap: " << t.timing_recWAMPF.timing_PIOW_overlapping.time_check_overlap<< "\n"
      "        num_overlapping_windows: " << t.timing_recWAMPF.timing_PIOW_overlapping.num_overlapping_windows<< "\n"
      "        time_merge_windows: " << t.timing_recWAMPF.timing_PIOW_overlapping.time_merge_windows<< "\n"
      "        time_remove_window: " << t.timing_recWAMPF.timing_PIOW_overlapping.time_remove_window<< "\n"
      "        time_remove_window: " << t.timing_recWAMPF.timing_PIOW_overlapping.time_add_to_windows<< "\n"
      "        timing_PlanIn:\n"
      "            total_PlanIn:" << t.timing_recWAMPF.timing_PIOW_overlapping.timing_PlanIn.total_PlanIn << "\n"
      "            num_until_goal_expanded:" <<
      t.timing_recWAMPF.timing_PIOW_overlapping.timing_PlanIn.num_until_goal_expanded << "\n"
      "            timing_expansions:\n"
      "                total_S3Exp: " << t.timing_recWAMPF.timing_PIOW_overlapping.timing_PlanIn.timing_expansions.total_S3Exp<< "\n"
      "                time_top_o: " << t.timing_recWAMPF.timing_PIOW_overlapping.timing_PlanIn.timing_expansions.time_top_o<< "\n"
      "                time_check_in_c: " << t.timing_recWAMPF.timing_PIOW_overlapping.timing_PlanIn.timing_expansions.time_check_in_c<< "\n"
      "                time_add_parent: " << t.timing_recWAMPF.timing_PIOW_overlapping.timing_PlanIn.timing_expansions.time_add_parent<< "\n"
      "                num_neighbors: " << t.timing_recWAMPF.timing_PIOW_overlapping.timing_PlanIn.timing_expansions.num_neighbors<< "\n"
      "                time_add_neighbors_to_o: " << t.timing_recWAMPF.timing_PIOW_overlapping.timing_PlanIn.timing_expansions.time_add_neighbors_to_o<< "\n"
      "    num_new_collisions: " << t.timing_recWAMPF.num_new_collisions<< "\n"
      "    timing_PIOW_new_collisions:\n"
      "        total_PIOW: " << t.timing_recWAMPF.timing_PIOW_new_collisions.total_PIOW<< "\n"
      "        num_windows: " << t.timing_recWAMPF.timing_PIOW_new_collisions.num_windows<< "\n"
      "        time_check_overlap: " << t.timing_recWAMPF.timing_PIOW_new_collisions.time_check_overlap<< "\n"
      "        num_overlapping_windows: " << t.timing_recWAMPF.timing_PIOW_new_collisions.num_overlapping_windows<< "\n"
      "        time_merge_windows: " << t.timing_recWAMPF.timing_PIOW_new_collisions.time_merge_windows<< "\n"
      "        time_remove_window: " << t.timing_recWAMPF.timing_PIOW_new_collisions.time_remove_window<< "\n"
      "        time_remove_window: " << t.timing_recWAMPF.timing_PIOW_new_collisions.time_add_to_windows << "\n"
      "        timing_PlanIn:\n"
      "            total_PlanIn:" << t.timing_recWAMPF.timing_PIOW_new_collisions.timing_PlanIn.total_PlanIn << "\n"
      "            num_until_goal_expanded:" <<
      t.timing_recWAMPF.timing_PIOW_new_collisions.timing_PlanIn.num_until_goal_expanded << "\n"
      "            timing_expansions:\n"
      "                total_S3Exp: " << t.timing_recWAMPF.timing_PIOW_new_collisions.timing_PlanIn.timing_expansions.total_S3Exp<< "\n"
      "                time_top_o: " << t.timing_recWAMPF.timing_PIOW_new_collisions.timing_PlanIn.timing_expansions.time_top_o<< "\n"
      "                time_check_in_c: " << t.timing_recWAMPF.timing_PIOW_new_collisions.timing_PlanIn.timing_expansions.time_check_in_c<< "\n"
      "                time_add_parent: " << t.timing_recWAMPF.timing_PIOW_new_collisions.timing_PlanIn.timing_expansions.time_add_parent<< "\n"
      "                num_neighbors: " << t.timing_recWAMPF.timing_PIOW_new_collisions.timing_PlanIn.timing_expansions.num_neighbors<< "\n"
      "                time_add_neighbors_to_o: " << t.timing_recWAMPF.timing_PIOW_new_collisions.timing_PlanIn.timing_expansions.time_add_neighbors_to_o<< "\n"
      "    num_windows_should_quit: " << t.timing_recWAMPF.num_windows_should_quit << "\n"
      "    time_should_quit: " << t.timing_recWAMPF.time_should_quit << "\n"
      "    time_remove_window: " << t.timing_recWAMPF.time_remove_window << "\nComplete!"
      ;
      return os;
    }
    // clang-format on
  };
  using TimingWAMPF = TimingWAMPF_t<Timer>;

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

  using AgentIdxs_t = std::vector<size_t>;
  using JointState_t = std::vector<State>;
  using JointAction_t = std::vector<Action>;
  using JointCost_t = std::vector<Cost>;
  using LowLevelSearch_t = AStar<State, Action, Cost, LowLevelEnvironment>;
  using IndividualPlan_t = PlanResult<State, Action, Cost>;
  using JointPlan_t = std::vector<IndividualPlan_t>;
  using JointNeighbor_t = std::vector<Neighbor<State, Action, Cost>>;

  static void print(const JointPlan_t& p, std::ostream& os = std::cout,
                    const std::string end = "\n") {
    for (size_t i = 0; i < p.size(); ++i) {
      const auto& e = p[i];
      os << "i: " << i << " | ";
      assert(e.states.size() == e.actions.size() + 1);
      os << e.states.front().first << ' ' << e.states.front().second << ' ';
      for (size_t j = 0; j < e.actions.size(); ++j) {
        const State& s = e.states.at(j + 1).first;
        const Cost& c = e.states.at(j + 1).second;
        const Action& a = e.actions.at(j).first;
        os << a << ' ' << s << ' ' << c << ' ';
      }
      os << end;
    }
    os << end;
  }

  template <typename T>
  static void print(const std::vector<T>& ss, std::ostream& os = std::cout,
                    const std::string end = "\n") {
    for (const T& s : ss) {
      os << s << ' ';
    }
    os << end;
  }

  struct Node {
    Node(const JointState_t& state, const JointAction_t& action,
         const JointCost_t& action_cost, const JointState_t& prev_state,
         const Cost& f_score, const JointCost_t& g_score)
        : state(state),
          action(action),
          action_cost(action_cost),
          prev_state(prev_state),
          f_score(f_score),
          g_score_sum(utils::sum(g_score)),
          g_score(g_score) {}

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
      assert(node.state.size() == node.action.size());
      assert(utils::sum(node.g_score) == node.g_score_sum);
      os << "Node: f_score: " << node.f_score
         << " g_score: " << node.g_score_sum << " ";
      for (size_t i = 0; i < node.state.size(); ++i) {
        const State& s = node.state.at(i);
        const State& ps = node.prev_state.at(i);
        const Action& a = node.action.at(i);
        //         const Cost& c = node.g_score.at(i);
        os << "(" << ps << " =" << a << "> " << s << ") "
           << " ";
      }
      return os;
    }

    JointState_t state;
    JointAction_t action;
    JointCost_t action_cost;
    JointState_t prev_state;

    Cost f_score;
    Cost g_score_sum;
    JointCost_t g_score;

    typename boost::heap::d_ary_heap<Node, boost::heap::arity<2>,
                                     boost::heap::mutable_<true>>::handle_type
        handle;
  };

  struct ParentValue_t {
    JointState_t current_state;
    JointState_t previous_state;
    JointAction_t action_out_of_previous_state;
    JointCost_t cost_out_of_previous_state;
    JointCost_t g_score_previous_state;

    ParentValue_t() = default;
    ParentValue_t(const JointState_t& current_state,
                  const JointState_t& previous_state,
                  const JointAction_t& action_out_of_previous_state,
                  const JointCost_t& cost_out_of_previous_state,
                  const JointCost_t& g_score_previous_state)
        : current_state(current_state),
          previous_state(previous_state),
          action_out_of_previous_state(action_out_of_previous_state),
          cost_out_of_previous_state(cost_out_of_previous_state),
          g_score_previous_state(g_score_previous_state) {}
  };

  using open_set_t =
      typename boost::heap::d_ary_heap<Node, boost::heap::arity<2>,
                                       boost::heap::mutable_<true>>;
  using open_set_handle_t = typename open_set_t::handle_type;

  using state_to_heap_t =
      std::unordered_map<JointState_t, open_set_handle_t, StateHasher>;

  using closed_set_t =
      std::unordered_map<JointState_t, JointCost_t, StateHasher>;

  using ParentMap_t =
      std::unordered_map<JointState_t, ParentValue_t, StateHasher>;

  using out_of_window_t = std::unordered_map<JointState_t, Node, StateHasher>;

  using goal_wait_nodes_t = std::vector<Node>;

  struct SearchState {
    static constexpr Cost kDefaultCost = -1;
    bool disabled;
    Cost min_cost;
    Cost max_cost;
    open_set_t open_set;
    state_to_heap_t state_to_heap;
    closed_set_t closed_set;
    ParentMap_t parent_map;
    out_of_window_t out_of_window;
    goal_wait_nodes_t goal_wait_nodes;
    JointState_t previous_start;

    SearchState()
        : disabled(false),
          min_cost(kDefaultCost),
          max_cost(kDefaultCost),
          open_set(),
          state_to_heap(),
          closed_set(),
          parent_map(),
          out_of_window(),
          goal_wait_nodes(),
          previous_start() {}

    bool operator==(const SearchState& o) const {
      return disabled == o.disabled && min_cost == o.min_cost &&
             max_cost == o.max_cost && open_set == o.open_set &&
             state_to_heap == o.state_to_heap && closed_set == o.closed_set &&
             parent_map == o.parent_map && out_of_window == o.out_of_window &&
             goal_wait_nodes == o.goal_wait_nodes &&
             previous_start == o.previous_start;
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

    void disable() {
      static constexpr bool kEagerMemoryClearing = false;
      disabled = true;
      if (kEagerMemoryClearing) {
        open_set.clear();
        state_to_heap.clear();
        closed_set.clear();
        parent_map.clear();
        out_of_window.clear();
        goal_wait_nodes.clear();
      }
    }

    friend std::ostream& operator<<(std::ostream& os, const SearchState& ps) {
      os << "Enabled: " << (ps.enabled ? "true" : "false");
      return os;
    }
  };

  using SSListIndex_t = size_t;
  using SSList_t = utils::StableStorage<SearchState, 5000>;

  struct WindowPlannerState {
    Window window;
    SSListIndex_t ss_index;
    SSList_t* ss_list;
    JointState_t initial_starts;
    JointState_t final_goal;

    WindowPlannerState() = delete;
    WindowPlannerState(const Window& window, 
                       SSList_t* ss_list, 
                       const JointState_t& initial_starts, 
                       const JointState_t& final_goal)
        : window(window), 
        ss_index(ss_list->add({})), 
        ss_list(ss_list), 
        initial_starts(initial_starts), 
        final_goal(final_goal) {}

    bool operator==(const WindowPlannerState& o) const {
      return (window == o.window) && (ss_index == o.ss_index) &&
             (ss_list == o.ss_list);
    }
    
    bool isInitialStartInWindow() const {
      for (const size_t& idx : window.agent_idxs) {
        const State& s = initial_starts.at(idx);
        if (!window.contains(s)) {
          return false;
        }
      }
      return true;
    }
    
    bool isFinalGoalInWindow() const {
      for (const size_t& idx : window.agent_idxs) {
        const State& s = final_goal.at(idx);
        if (!window.contains(s)) {
          return false;
        }
      }
      return true;
    }

    WindowPlannerState merge(const WindowPlannerState& o) const {
      return {window.merge(o.window), ss_list, initial_starts, final_goal};
    }

    bool overlapping(const WindowPlannerState& other) const {
      // If no overlap in space, then they cannot be overlapping in paths.
      if (!window.intersects(other.window)) {
        return false;
      }

      const SearchState* ss = getSearchState();
      const SearchState* oss = other.getSearchState();
      if (ss->min_cost != SearchState::kDefaultCost || 
          ss->max_cost != SearchState::kDefaultCost || 
          oss->min_cost != SearchState::kDefaultCost || 
          oss->max_cost != SearchState::kDefaultCost) {
        // If they overlap in space and share agents, then they need to be merged.
        return window.has_overlapping_agents(other.window);
      } 

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
      os << "Window: " << ws.window
                << " Search state index: " << ws.ss_index << " starts: ";
      print(ws.initial_starts, os, " goals: ");
      print(ws.final_goal, os);
      return os;
    }
  };

  using WPS_t = WindowPlannerState;
  using WPSList_t = std::vector<WindowPlannerState>;

  void growAndMergeExisting(WPSList_t* windows, JointPlan_t* solution,
                            TimingRecWAMPF* timing_recWAMPF) {
    static constexpr bool kDebug = false;
    for (size_t i = 0; i < windows->size(); ++i) {
      timing_recWAMPF->num_windows++;
      WPS_t& wi = windows->at(i);
      bool gri_res = growAndReplanIn(&wi, solution, &(timing_recWAMPF->timing_gari));
      while (!gri_res) {
        wi.window.grow();
        TimingPlanIn tp;
        gri_res = planIn(&wi, solution, &tp);
      }

      timing_recWAMPF->timing_PIOW_overlapping.total_PIOW.start();
      bool found_overlapping = false;
      do {
        found_overlapping = false;
        timing_recWAMPF->timing_PIOW_overlapping.num_windows += windows->size();
        for (size_t j = 0; j < windows->size(); ++j) {
          if (i == j) {
            continue;
          }
          const WPS_t& wj = windows->at(j);
          timing_recWAMPF->timing_PIOW_overlapping.time_check_overlap.start();
          const bool is_overlapping = wi.overlapping(wj);
          timing_recWAMPF->timing_PIOW_overlapping.time_check_overlap.stop();
          if (is_overlapping) {
            found_overlapping = true;
            if (kDebug) {
              std::cout << "Merging two existing windows " << wi << ", " << wj
                        << std::endl;
            }
            timing_recWAMPF->timing_PIOW_overlapping.num_overlapping_windows++;
            timing_recWAMPF->timing_PIOW_overlapping.time_merge_windows.start();
            wi.getSearchState()->disable();
            wj.getSearchState()->disable();
            timing_recWAMPF->timing_PIOW_overlapping.time_add_to_windows
                .start();
            wi = wi.merge(wj);
            timing_recWAMPF->timing_PIOW_overlapping.time_add_to_windows.stop();
            timing_recWAMPF->timing_PIOW_overlapping.time_merge_windows.stop();
            while (!planIn(
                &wi, solution,
                &(timing_recWAMPF->timing_PIOW_overlapping.timing_PlanIn))) {
              wi.window.grow();
            }

            timing_recWAMPF->timing_PIOW_overlapping.time_remove_window.start();
            // Erase wj.
            windows->erase(windows->begin() + j);
            timing_recWAMPF->timing_PIOW_overlapping.time_remove_window.stop();

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
      timing_recWAMPF->timing_PIOW_overlapping.total_PIOW.stop();
    }
  }

  void integrateNewConflictWindow(WPS_t window, WPSList_t* windows,
                                  JointPlan_t* solution,
                                  TimingPIOW* timing_PIOW) {
    timing_PIOW->total_PIOW.start();
    
    bool found_overlapping = false;
    do {
      found_overlapping = false;
      for (size_t i = 0; i < windows->size(); ++i) {
        timing_PIOW->num_windows++;
        const WPS_t& wi = windows->at(i);
        timing_PIOW->time_check_overlap.start();
        const bool is_overlapping = window.overlapping(wi);
        timing_PIOW->time_check_overlap.stop();
        if (is_overlapping) {
          found_overlapping = true;
          timing_PIOW->num_overlapping_windows++;
          timing_PIOW->time_merge_windows.start();
          window.getSearchState()->disable();
          wi.getSearchState()->disable();
          window = window.merge(wi);
          wi.getSearchState()->disable();
          windows->erase(windows->begin() + i);
          timing_PIOW->time_remove_window.stop();
          break;
        }
      }
    } while (found_overlapping);
   
    while (!planIn(&window, solution, &(timing_PIOW->timing_PlanIn))) {
      window.window.grow();
    }
    
    found_overlapping = false;
    do {
      found_overlapping = false;
      for (size_t i = 0; i < windows->size(); ++i) {
        timing_PIOW->num_windows++;
        const WPS_t& wi = windows->at(i);
        timing_PIOW->time_check_overlap.start();
        const bool is_overlapping = window.overlapping(wi);
        timing_PIOW->time_check_overlap.stop();
        if (is_overlapping) {
          found_overlapping = true;
          timing_PIOW->num_overlapping_windows++;
          timing_PIOW->time_merge_windows.start();
          window.getSearchState()->disable();
          wi.getSearchState()->disable();
          window = window.merge(wi);
          timing_PIOW->time_merge_windows.stop();
          while (!planIn(&window, solution, &(timing_PIOW->timing_PlanIn))) {
            window.window.grow();
          }
          timing_PIOW->time_remove_window.start();
          wi.getSearchState()->disable();
          windows->erase(windows->begin() + i);
          timing_PIOW->time_remove_window.stop();
          break;
        }
      }
    } while (found_overlapping);
    timing_PIOW->time_add_to_windows.start();
    windows->push_back(window);
    timing_PIOW->time_add_to_windows.stop();
    timing_PIOW->total_PIOW.stop();
  }

  void removeCompletedWindows(WPSList_t* windows,
                              TimingRecWAMPF* timing_recWAMPF) {
    for (size_t i = 0; i < windows->size();) {
      WPS_t& w = (*windows)[i];
      timing_recWAMPF->time_should_quit.start();
      const bool was_restricted = w.getSearchState()->wasSearchRestricted();
      const bool start_in_window = w.isInitialStartInWindow();
      const bool goal_in_window = w.isFinalGoalInWindow();
      static int iter = 0;
      const bool iter_done = false; ++iter > 1;
//       std::cout << "USING ITER\n";
      timing_recWAMPF->time_should_quit.stop();
      if ((!was_restricted && start_in_window && goal_in_window) || iter_done) {
        timing_recWAMPF->num_windows_should_quit++;
        timing_recWAMPF->time_remove_window.start();
        windows->erase(windows->begin() + i);
        timing_recWAMPF->time_remove_window.stop();
      } else {
        ++i;
      }
    }
  }

  bool recWAMPF(const JointState_t& initial_starts, 
                const JointState_t& final_goal,
                WPSList_t* windows, SSList_t* search_states,
                JointPlan_t* solution, TimingRecWAMPF* timing_recWAMPF) {
    timing_recWAMPF->total_RecWAMPF.start();
    static constexpr bool kDebug = false;
    if (kDebug) {
      std::cout << "Starting recWAMPF\n";
    }
    growAndMergeExisting(windows, solution, timing_recWAMPF);

    Conflict result;
    while (m_env.getFirstConflict(*solution, result)) {
      timing_recWAMPF->num_new_collisions++;
      if (kDebug) {
        std::cout << "Found first conflict\n";
        std::cout << "Conflict: " << result << std::endl;
      }
      WPS_t window(m_env.createWindowFromConflict(result), search_states, initial_starts, final_goal);
      integrateNewConflictWindow(window, windows, solution,
                                 &timing_recWAMPF->timing_PIOW_new_collisions);
    }
    
    if (kDebug) {
      std::cout << "Conflict resolution done!\n";
    }

    for (const WPS_t& w : *windows) {
      timing_recWAMPF->num_max_agents_in_window =
          std::max(timing_recWAMPF->num_max_agents_in_window,
                   w.window.agent_idxs.size());
    }

    timing_recWAMPF->num_windows += windows->size();
    removeCompletedWindows(windows, timing_recWAMPF);
    if (kDebug) {
      std::cout << "Finished all conflicts\n";
      Cost cost = 0;
      for (const auto& is : *solution) {
        cost += is.cost;
      }
      std::cout << "Cost: " << cost << "\n";
      print(*solution);
    }
    timing_recWAMPF->total_RecWAMPF.stop();
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

  void popOpenSetTop(open_set_t* open_set, state_to_heap_t* state_to_heap) {
    state_to_heap->erase(open_set->top().state);
    open_set->pop();
  }

  bool AStarSearchUntil(WPS_t* window, const JointState_t& starts,
                        const JointState_t& goals, const JointPlan_t& solution,
                        const Cost& fmax,
                        TimingAStarSearchUntil* timing_AStarSearchUntil) {
    verifyOpenSet(*window, solution);
    timing_AStarSearchUntil->total_astar_search_until.start();
    static constexpr bool kDebug = false;
    assert(!(window->window.agent_idxs.empty()));

    auto* ss = window->getSearchState();
    open_set_t& open_set = ss->open_set;
    state_to_heap_t& state_to_heap = ss->state_to_heap;
    closed_set_t& closed_set = ss->closed_set;
    ParentMap_t& parent_map = ss->parent_map;
    out_of_window_t& out_of_window = ss->out_of_window;
    goal_wait_nodes_t& goal_wait_nodes = ss->goal_wait_nodes;

    assert(!open_set.empty());

    //     std::cout << "Starts: ";
    //     for (const auto& s : starts) {
    //       std::cout << s << ' ';
    //     }
    //     std::cout << std::endl;
    //
    //     std::cout << "Goals: ";
    //     for (const auto& s : goals) {
    //       std::cout << s << ' ';
    //     }
    //     std::cout << std::endl;

    timing_AStarSearchUntil->timing_ASUExp.total_ASUExp.start();
    for (size_t expand_count = 0;
         !open_set.empty() && open_set.top().f_score < fmax; ++expand_count) {
      verifyOpenSet(*window, solution);
      if (isTooManyIterations(expand_count, *window)) {
        return false;
      }
      timing_AStarSearchUntil->num_values_f_less_fmax++;

      Node current = open_set.top();
      if (kDebug) {
        std::cout << current << std::endl;
      }
      bool is_debug_state = isDebugState(current);

      assert(current.state.size() == current.action.size());

      //       for (size_t i = 0; i < current.state.size(); ++i) {
      //         const auto& s = current.state.at(i);
      //         const auto& a = current.action.at(i);
      //         const auto& c = current.g_score.at(i);
      //         std::cout << s << ' ' << a << ' ' << c << ' ';
      //       }
      //       std::cout << std::endl;

      assert(window->window.agent_idxs.size() == current.state.size());

      m_env.onExpandNode(current.state, current.f_score, current.g_score);

      timing_AStarSearchUntil->timing_ASUExp.time_top_o.start();
      popOpenSetTop(&open_set, &state_to_heap);
      timing_AStarSearchUntil->timing_ASUExp.time_top_o.stop();

      timing_AStarSearchUntil->timing_ASUExp.time_check_in_c.start();
      
      if (inClosedSetAndClosed(closed_set, current.state, current.g_score)) {
        if (kDebug) {
          std::cout << "Closed, skipping " << current << std::endl;
        }
        timing_AStarSearchUntil->timing_ASUExp.time_check_in_c.stop();
        continue;
      }
      insertClosedSet(&closed_set, current.state, current.g_score);
      timing_AStarSearchUntil->timing_ASUExp.time_check_in_c.stop();

      if (kDebug) {
        std::cout << "A*SU Exp: " << current << std::endl;
      }

      timing_AStarSearchUntil->timing_ASUExp.time_add_parent.start();
      insertParentMap(&parent_map, current);
      timing_AStarSearchUntil->timing_ASUExp.time_add_parent.stop();

      auto neighbor_generator = m_env.getJointWindowNeighbors(
          current.state, current.action, goals, window->window, solution);

      JointNeighbor_t joint_neighbor_info(current.state.size());
      bool joint_neighbor_in_window = true;
      bool neighbor_has_goal_wait = false;
      timing_AStarSearchUntil->timing_ASUExp.time_add_neighbors_to_o.start();
      while (!neighbor_generator.atEnd()) {
        timing_AStarSearchUntil->timing_ASUExp.num_neighbors++;
        const auto& generator_output = neighbor_generator.getAndIncrement();
        extractJointNeighborInfo(&joint_neighbor_info,
                                 &joint_neighbor_in_window, generator_output);

        neighbor_has_goal_wait =
            neighbor_has_goal_wait || containsGoalWait(joint_neighbor_info);

        //         std::cout << "Neighbor: ";
        //         for (const auto& n : joint_neighbor_info) {
        //           std::cout << n.state << ' ' << n.action << ' ' << n.cost <<
        //           ' ';
        //         }
        //         std::cout << std::endl;

        for (size_t i = 0; i < current.action.size(); ++i) {
          const auto& n = joint_neighbor_info.at(i);
          if (current.action.at(i) == Action::GoalWait) {
            assert(n.action == Action::GoalWait);
          }
        }
        ProcessNeighborResult r = processNeighbor(starts, goals, current, joint_neighbor_info,
                        joint_neighbor_in_window, closed_set, &state_to_heap,
                        &open_set, &out_of_window);
//         if (is_debug_state) {
//           std::cout << r << std::endl;
//         }
      }

      timing_AStarSearchUntil->timing_ASUExp.time_add_neighbors_to_o.stop();

      if (neighbor_has_goal_wait) {
        goal_wait_nodes.push_back(current);
      }
    }
    timing_AStarSearchUntil->timing_ASUExp.total_ASUExp.stop();
    timing_AStarSearchUntil->total_astar_search_until.stop();
    return true;
  }

  inline void verifyOpenSet(const WPS_t& window, const JointPlan_t&) {
    if (kProduction) {
      return;
    }
    for (const Node& n : window.getSearchState()->open_set) {
      assert(n.state.size() == window.window.agent_idxs.size());
    }
  }

  inline void verifyParentMap(const ParentMap_t& parent_map,
                              const JointState_t& starts) {
    if (kProduction) {
      return;
    }
    assert(!starts.empty());
    assert(parent_map.find(toParentMapKey(starts)) != parent_map.end());
  }

  bool Stage1(WPS_t* window, const JointState_t& starts,
              const JointState_t& goals, const JointPlan_t& solution,
              const Cost& goal_node_fvalue, TimingStage1* timing_stage1) {
    timing_stage1->total_Stage1.start();
    SearchState* ss = window->getSearchState();

    verifyParentMap(ss->parent_map, ss->previous_start);

    verifyOpenSet(*window, solution);
    timing_stage1->time_add_x_to_o.start();
    for (const std::pair<JointState_t, Node>& sn : ss->out_of_window) {
      const Node& n = sn.second;
      const JointState_t& current_state = n.prev_state;
      const JointState_t& neighbor_state = n.state;
      const JointAction_t& neighbor_action = n.action;
      const JointCost_t& neighbor_action_cost = n.action_cost;
      const JointCost_t& neighbor_g_score = n.g_score;
      insertStateIntoOpen(goals, current_state, neighbor_state, neighbor_action,
                          neighbor_action_cost, neighbor_g_score,
                          &(ss->state_to_heap), &(ss->open_set));
    }
    timing_stage1->time_add_x_to_o.stop();
    timing_stage1->time_clear_x.start();
    ss->out_of_window.clear();
    timing_stage1->time_clear_x.stop();

    verifyOpenSet(*window, solution);

    bool res = AStarSearchUntil(window, starts, goals, solution, goal_node_fvalue,
                     &timing_stage1->timing_AStarSearchUntil);
    timing_stage1->total_Stage1.stop();
    return res;
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

    void verify(const WPS_t& window, const JointState_t& old_starts,
                const JointState_t& new_starts) const {
      if (kProduction) {
        return;
      }
      static constexpr bool kDebug = false;
      assert(!cost_between_starts.empty());

      for (const Node& n : nodes) {
        assert(window.window.agent_idxs.size() == n.state.size());
        if (kDebug) {
          std::cout << "Node state: ";
          print(n.state);
        }
      }

      assert(nodes.front().state == new_starts);
      assert(nodes.back().state == old_starts);
    }
  };

  bool Stage2(WPS_t* window, const NodeAndCameFromValues_t& info_between_starts,
              const JointState_t& starts, const JointState_t& goals,
              const JointPlan_t& solution, const Cost& goal_node_fvalue,
              TimingStage2* timing_stage2) {
    timing_stage2->total_Stage2.start();
    static constexpr bool kDebug = false;
    SearchState* ss = window->getSearchState();

    verifyOpenSet(*window, solution);
    verifyParentMap(ss->parent_map, ss->previous_start);

    eraseParentMap(&(ss->parent_map), ss->previous_start);
    eraseClosedSet(&(ss->closed_set), ss->previous_start);
    ss->previous_start = starts;

    // Insert path into openlist.
    timing_stage2->time_expanding_s.start();
    for (const Node& node : info_between_starts.nodes) {
      timing_stage2->num_len_path++;
      assert(node.state.size() == window->window.agent_idxs.size());
      Node node_copy = node;
      // Hack to make sure that the node is expanded before any other nodes.
      node_copy.f_score = node_copy.f_score / 2;
      auto handle = ss->open_set.push(node_copy);
      (*handle).handle = handle;
      ss->state_to_heap[node_copy.state] = handle;
      if (kDebug) {
        std::cout << "Node state pushed: " << node_copy << " top fval: " << ss->open_set.top().f_score << std::endl;
      }
    }
    timing_stage2->time_expanding_s.stop();

    verifyOpenSet(*window, solution);
    if (kDebug) {
      std::cout << "Astar search until " << goal_node_fvalue << "\n";
    }
    bool res = AStarSearchUntil(window, starts, goals, solution, goal_node_fvalue,
                          &timing_stage2->timing_AStarSearchUntil);
    timing_stage2->total_Stage2.stop();
    return res;
  }

  void verifySolutionValid(const JointPlan_t& solution) {
    if (kProduction) {
      return;
    }
    static constexpr bool kDebug = false;
    if (kDebug) {
      std::cout << "Verifying solution: " << std::endl;
      print(solution);
    }
    for (const auto& p : solution) {
      assert(!p.states.empty());
      assert(p.states.size() == p.actions.size() + 1);

      for (size_t i = 1; i < p.states.size(); ++i) {
        const State& prev_s = p.states.at(i - 1).first;
        const Cost& prev_c = p.states.at(i - 1).second;
        const State& curr_s = p.states.at(i).first;
        const Cost& curr_c = p.states.at(i).second;
        if (!(prev_s.time + 1 == curr_s.time)) {
          std::cout << "ERR: path cost is wrong!";
        }
        assert(prev_s.time <= curr_s.time);
        assert(prev_s.time + 1 == curr_s.time);
        if (!(prev_c + 1 == curr_c || prev_c == curr_c)) {
          std::cout << "Current s: " << curr_s << " Current c: " << curr_c
                    << std::endl;
          std::cout << "Prev s: " << prev_s << " Previous c: " << prev_c
                    << std::endl;
        }
        assert(prev_c + 1 == curr_c || prev_c == curr_c);
      }

      for (size_t i = 1; i < p.actions.size(); ++i) {
        const auto& curr_action = p.actions.at(i);
        const auto& prev_action = p.actions.at(i - 1);
        if (prev_action.first == Action::GoalWait) {
          assert(curr_action.first == Action::GoalWait);
        }
      }
    }
  }

  void recomputeHeuristic(open_set_t* open_set, state_to_heap_t* state_to_heap,
                          const JointState_t& goals) {
    for (auto& sh : *state_to_heap) {
      auto& handle = sh.second;
      assert((*handle).state == sh.first);
      if ((*handle).f_score > 0) {
        (*handle).f_score =
            (*handle).g_score_sum +
            m_env.admissibleJointHeuristic((*handle).state, goals);
        open_set->update(sh.second);
      }
    }
  }

  void removeGoalWaitMovesClosed(closed_set_t* closed_set,
                                 ParentMap_t* parent_map) {
    static constexpr bool kDebug = false;
    for (auto it = parent_map->begin(); it != parent_map->end();) {
      std::pair<const JointState_t, ParentValue_t>& pair = *it;
      const JointState_t& s = pair.first;
      ParentValue_t& v = pair.second;
      if (containsGoalWait(v.action_out_of_previous_state)) {
        auto closed_prev_it = closed_set->find(toParentMapKey(v.current_state));
        if (kDebug) {
          std::cout << "DELETING ";
          print(v.previous_state, std::cout, " ");
          print(v.action_out_of_previous_state, std::cout, " => ");
          print(s);
        }
        if (closed_prev_it != closed_set->end()) {
          closed_set->erase(closed_prev_it);
        }
        it = parent_map->erase(it);
      } else {
        it++;
      }
    }
  }

  void removeGoalWaitMovesOpen(open_set_t* open_set,
                               state_to_heap_t* state_to_heap) {
    for (auto it = state_to_heap->begin(); it != state_to_heap->end();) {
      auto& handle = it->second;
      if (containsGoalWait((*handle).action)) {
        open_set->erase(handle);
        it = state_to_heap->erase(it);
      } else {
        ++it;
      }
    }
  }

  void readdGoalWaitNodes(open_set_t* open_set, state_to_heap_t* state_to_heap,
                          goal_wait_nodes_t* goal_wait_nodes,
                          closed_set_t* closed_set, ParentMap_t* parent_map, const JointState_t& goals) {
    static constexpr bool kDebug = false;
    for (Node& n : *goal_wait_nodes) {
      if (containsGoalWait(n.action)) {
        continue;
      }
      n.f_score = m_env.admissibleJointHeuristic(n.state, goals) + n.g_score_sum;
      if (kDebug) {
        std::cout << "READDING WAIT NODE: " << n << std::endl;
      }
      auto handle = open_set->push(n);
      (*handle).handle = handle;
      state_to_heap->insert(std::make_pair<>(n.state, handle));

      eraseClosedSet(closed_set, n.state);
    }
    goal_wait_nodes->clear();
  }

//   bool containsState(const Node& n, const State& s, const size_t idx) {
//     if (n.state.size() <= idx) {
//       return false;
//     }
//     return n.state.at(idx) == (s);
//   }
//   
//   bool containsState(const JointNeighbor_t& n, const State& s, const size_t idx) {
//     if (n.size() <= idx) {
//       return false;
//     }
//     return n.at(idx).state == (s);
//   }

  void checkClosedSetParentMapDebugState(const closed_set_t& closed_set, const ParentMap_t& parent_map) {
    auto k = toParentMapKey( JointState_t({ {0, 2, 2}, {0, 1, 2} }));
    auto c_res = closed_set.find(k);
    auto p_res = parent_map.find(k);
    if (c_res == closed_set.end()) {
      if (p_res != parent_map.end()) {
        std::cout << "In parent map but not closed: ";
        print(k);
      }
//       assert(p_res == parent_map.end());
      std::cout << "-----------------------------------Debug state not in closed set!" << std::endl;
      return;
    }
    assert(p_res != parent_map.end());
    
    std::cout << "-----------------------------------Result found with cost: ";
    print(c_res->second);
    std::cout << "Parents: ";
    print(p_res->second.current_state, std::cout, "<= "); 
    print(p_res->second.previous_state);
  }

  bool isDebugState(const Node& current) {
    bool is_debug_state = false;
    if (toParentMapKey(current.state) == JointState_t({ {0, 2, 2}, {0, 1, 2} })) {
        is_debug_state = true;
        std::cout << std::endl;
        std::cout << current << std::endl;
        std::cout << "0000000000000000000000000000000000000000000000000000000000000000000" << std::endl;
        std::cout << std::endl;
        std::cout << "G-score: ";
        print(current.g_score);
    }
    
    if (toParentMapKey(current.state) == JointState_t({ {0, 3, 2}, {0, 1, 1} })) {
        is_debug_state = true;
        std::cout << std::endl;
        std::cout << current << std::endl;
        std::cout << "D1D1D1D1D1D1D1D1D1D1D1D1D1D1D1D1D1D1D1D1D1D1D1D1D1D1D1D1D1D1D1D1D1" << std::endl;
        std::cout << std::endl;
        std::cout << "G-score: ";
        print(current.g_score);
    }
    
    if (toParentMapKey(current.state) == JointState_t({ {0, 3, 2}, {0, 1, 0} })) {
        is_debug_state = true;
        std::cout << std::endl;
        std::cout << current << std::endl;
        std::cout << "D2D2D2D2D2D2D2D2D2D2D2D2D2D2D2D2D2D2D2D2D2D2D2D2D2D2D2D2D2D2D2D2D2" << std::endl;
        std::cout << std::endl;
        std::cout << "G-score: ";
        print(current.g_score);
    }
    
    if (toParentMapKey(current.state) == JointState_t({ {0, 3, 2}, {0, 2, 0} })) {
        is_debug_state = true;
        std::cout << std::endl;
        std::cout << current << std::endl;
        std::cout << "D3D3D3D3D3D3D3D3D3D3D3D3D3D3D3D3D3D3D3D3D3D3D3D3D3D3D3D3D3D3D3D3D3" << std::endl;
        std::cout << std::endl;
        std::cout << "G-score: ";
        print(current.g_score);
    }
    
    
    return is_debug_state;
  }
  
  bool Stage3(WPS_t* window, JointPlan_t* solution, const JointState_t& starts,
              const JointCost_t& starts_costs, const JointState_t& goals,
              const JointCost_t& goals_costs, const JointState_t& old_goals, TimingStage3* timing_stage3) {
    timing_stage3->total_Stage3.start();
    static constexpr bool kDebug = false;
    assert(!(window->window.agent_idxs.empty()));
    if (kDebug) {
      std::cout << "Stage 3 in: " << window->window << '\n';
      assert(starts.size() == starts_costs.size());
      for (size_t i = 0; i < starts.size(); ++i) {
        std::cout << starts.at(i) << ' ' << starts_costs.at(i) << " ";
      }
      std::cout << std::endl;
    }

    auto* ss = window->getSearchState();
    open_set_t& open_set = ss->open_set;
    state_to_heap_t& state_to_heap = ss->state_to_heap;
    closed_set_t& closed_set = ss->closed_set;
    ParentMap_t& parent_map = ss->parent_map;
    out_of_window_t& out_of_window = ss->out_of_window;
    goal_wait_nodes_t& goal_wait_nodes = ss->goal_wait_nodes;

    checkClosedSetParentMapDebugState(closed_set, parent_map);
    
    verifyParentMap(parent_map, ss->previous_start);

    checkClosedSetParentMapDebugState(closed_set, parent_map);
    
    readdGoalWaitNodes(&open_set, &state_to_heap, &goal_wait_nodes,
                       &closed_set, &parent_map, goals);
    std::cout << "readdGoalWaitNodes done" << std::endl;
    checkClosedSetParentMapDebugState(closed_set, parent_map);
    
    removeGoalWaitMovesClosed(&closed_set, &parent_map);
    eraseClosedSet(&closed_set, old_goals);
    eraseParentMap(&parent_map, old_goals);
    
    checkClosedSetParentMapDebugState(closed_set, parent_map);
    
    removeGoalWaitMovesOpen(&open_set, &state_to_heap);
    
    checkClosedSetParentMapDebugState(closed_set, parent_map);
    
    recomputeHeuristic(&open_set, &state_to_heap, goals);
    
    std::cout << "END!\n";
    checkClosedSetParentMapDebugState(closed_set, parent_map);

    assert(parent_map.find(toParentMapKey(starts)) != parent_map.end());
    
    if (kDebug) {
      std::cout << "Goals\n";
      print(goals);
    }
    
    if (inClosedSet(closed_set, goals)) {
      std::cout << "Goal closed already, unwinding!" << std::endl;
      assert(parent_map.find(toParentMapKey(goals)) != parent_map.end());
      
      const JointCost_t goals_unwind_cost = closed_set.find(toParentMapKey(goals))->second;
      
      auto window_solution =
            unwindPath(starts, starts_costs, goals, goals_unwind_cost, parent_map);
      if (kDebug) {
        std::cout << "Verifying unwound result" << std::endl;
      }
      verifySolutionValid(window_solution);

      const auto min_max = getMinMaxCostFromPlan(window_solution);
      for (const auto& e : starts_costs) {
        assert(min_max.first == e);
      }
      ss->min_cost = min_max.first;
      ss->max_cost = min_max.second;

      if (kDebug) {
        std::cout << "Inserting path into window" << std::endl;
      }
      bool insert_result =
          insertWindowPath(&window_solution, window->window, starts_costs,
                            goals_costs, solution);
      assert(insert_result);
      timing_stage3->timing_S3Exp.total_S3Exp.stop();
      timing_stage3->total_Stage3.stop();
      return true;
      
    }

    timing_stage3->timing_S3Exp.total_S3Exp.start();
    for (size_t expand_count = 0; !open_set.empty(); ++expand_count) {
      timing_stage3->num_until_goal_expanded++;
      if (isTooManyIterations(expand_count, *window)) {
        return false;
      }
      assert(!isTooManyIterations(expand_count, *window));

      Node current = open_set.top();

      if (kDebug) {
      std::cout << current << std::endl;
      }
      
      bool is_debug_state = isDebugState(current);
      
      
      
      m_env.onExpandNode(current.state, current.f_score, current.g_score);

      if (m_env.isJointSolution(current.state, goals)) {
        if (kDebug) {
          std::cout << "Goal node has an fvalue of " << current.f_score << std::endl;
        }
        insertParentMap(&parent_map, current);
        insertClosedSet(&closed_set, current.state, current.g_score);
        if (kDebug) {
          std::cout << "Starting unwind!" << std::endl;
        }
        auto window_solution =
            unwindPath(starts, starts_costs, current, parent_map);
        if (kDebug) {
          std::cout << "Verifying unwound result" << std::endl;
        }
        verifySolutionValid(window_solution);

        const auto min_max = getMinMaxCostFromPlan(window_solution);
        for (const auto& e : starts_costs) {
          assert(min_max.first == e);
        }
        ss->min_cost = min_max.first;
        ss->max_cost = min_max.second;

        if (kDebug) {
          std::cout << "Inserting path into window" << std::endl;
          std::cout << "Goal f_score: " << current.f_score << std::endl;
        }
        bool insert_result =
            insertWindowPath(&window_solution, window->window, starts_costs,
                             goals_costs, solution);
        assert(insert_result);
        timing_stage3->timing_S3Exp.total_S3Exp.stop();
        timing_stage3->total_Stage3.stop();
        return true;
      }

      timing_stage3->timing_S3Exp.time_top_o.start();
      popOpenSetTop(&open_set, &state_to_heap);
      timing_stage3->timing_S3Exp.time_top_o.stop();

      timing_stage3->timing_S3Exp.time_check_in_c.start();
      if (inClosedSetAndClosed(closed_set, current.state, current.g_score)) {
          if (kDebug || is_debug_state) {
            std::cout << "Closed, skipping " << current << std::endl;
          }
          timing_stage3->timing_S3Exp.time_check_in_c.stop();
          continue;
      }
      insertClosedSet(&closed_set, current.state, current.g_score);
      timing_stage3->timing_S3Exp.time_check_in_c.stop();

      timing_stage3->timing_S3Exp.time_add_parent.start();
      insertParentMap(&parent_map, current);
      timing_stage3->timing_S3Exp.time_add_parent.stop();

      auto neighbor_generator = m_env.getJointWindowNeighbors(
          current.state, current.action, goals, window->window, *solution);

      JointNeighbor_t joint_neighbor_info(current.state.size());
      bool joint_neighbor_in_window = true;
      bool neighbor_has_goal_wait = false;
      timing_stage3->timing_S3Exp.time_add_neighbors_to_o.start();
      if (is_debug_state) {
        std::cout << "Neighbors:" << std::endl;
      }
      while (!neighbor_generator.atEnd()) {
        timing_stage3->timing_S3Exp.num_neighbors++;
        const auto& generator_output = neighbor_generator.getAndIncrement();
        extractJointNeighborInfo(&joint_neighbor_info,
                                 &joint_neighbor_in_window, generator_output);

        neighbor_has_goal_wait =
            neighbor_has_goal_wait || containsGoalWait(joint_neighbor_info);

        for (size_t i = 0; i < current.action.size(); ++i) {
          const auto& n = joint_neighbor_info.at(i);
          if (current.action.at(i) == Action::GoalWait) {
            assert(n.action == Action::GoalWait);
          }
        }
        

//         const bool is_debug_state = toParentMapKey(current.state) == JointState_t({ {0, 2, 4}, {0, 3, 6}, {0, 5, 7} });
        ProcessNeighborResult r = 
          processNeighbor(starts, goals, current, joint_neighbor_info,
                          joint_neighbor_in_window, closed_set, &state_to_heap,
                          &open_set, &out_of_window, is_debug_state);
        if (is_debug_state) {
          std::cout << r << std::endl;
        }
//         if (is_debug_state) {
//           JointState_t neighbor_state;
//           JointAction_t neighbor_action;
//           for (const auto& n : joint_neighbor_info) {
//             neighbor_state.push_back(n.state);
//             neighbor_action.push_back(n.action);
//           }
//           std::cout << "N: ";
//           print(neighbor_state, std::cout << "Act: ");
//           print(neighbor_action);
//           switch(r) {
//             case ADDOPEN: { std::cout << "ADDOPEN\n"; break; }
//             case INCLOSED: { std::cout << "INCLOSED\n";break; }
//             case NOTINWINDOW: { std::cout << "NOTINWINDOW\n";break; }
//             case ISCOLLIDING: { std::cout << "ISCOLLIDING\n"; break;}
//           }
//         }
      }
      timing_stage3->timing_S3Exp.time_add_neighbors_to_o.stop();

      if (neighbor_has_goal_wait) {
        goal_wait_nodes.push_back(current);
      }
    }
    // Emptied openlist!
    timing_stage3->timing_S3Exp.total_S3Exp.stop();
    timing_stage3->total_Stage3.stop();
    return false;
  }

  void verifyStartCostDifference(const JointCost_t& before_starts_costs,
                                 const JointCost_t& after_starts_costs) {
    if (kProduction) {
      return;
    }
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
    static constexpr bool kDebug = false;

    if (kDebug) {
      std::cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> trimPlanBetweenCosts"
                << std::endl;

      std::cout << "Before starts: ";
      for (const auto& s : before_starts) {
        std::cout << s << ' ';
      }
      std::cout << std::endl;

      std::cout << "After starts: ";
      for (const auto& s : after_starts) {
        std::cout << s << ' ';
      }
      std::cout << std::endl;
    }

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

      p.actions.erase(p.actions.begin() + max_cost, p.actions.end());
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

    return std::make_tuple(between_starts_solution, between_starts_cost);
  }

  std::pair<JointState_t, JointCost_t> getIthFullState(
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

  std::pair<JointState_t, JointCost_t> getIthState(
      const JointPlan_t& joint_plan, const size_t i,
      const AgentIdxs_t& agent_idxs) {
    JointState_t s;
    JointCost_t c;
    for (const size_t& agent_idx : agent_idxs) {
      assert(agent_idx < joint_plan.size());
      const auto& p = joint_plan[agent_idx];
      assert(i < p.states.size());
      s.push_back(p.states[i].first);
      c.push_back(p.states[i].second);
    }
    return {s, c};
  }

  std::pair<JointAction_t, JointCost_t> getIthFullAction(
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

  std::pair<JointAction_t, JointCost_t> getIthAction(
      const JointPlan_t& joint_plan, const size_t i,
      const AgentIdxs_t& agent_idxs) {
    JointAction_t a;
    JointCost_t c;
    for (const size_t& agent_idx : agent_idxs) {
      assert(agent_idx < joint_plan.size());
      const auto& p = joint_plan[agent_idx];
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
    static constexpr bool kDebug = false;
    JointPlan_t between_starts_solution;
    NodeAndCameFromValues_t nodes_and_came_from;
    std::tie(between_starts_solution, nodes_and_came_from.cost_between_starts) =
        trimPlanBetweenCosts(window, solution, before_starts,
                             before_starts_costs, after_starts,
                             after_starts_costs);

    verifySolutionValid(between_starts_solution);

    if (kDebug) {
      std::cout << "Plan between starts\n";
      for (const auto& p : between_starts_solution) {
        assert(p.states.size() == p.actions.size() + 1);
        for (size_t i = 0; i < p.actions.size(); ++i) {
          const auto& s = p.states.at(i);
          std::cout << "(" << s.first << ' ' << s.second << ") "
                    << p.actions.at(i).first << " ";
        }
        std::cout << "(" << p.states.back().first << ' '
                  << p.states.back().second << ")" << std::endl;
      }
    }

    const size_t num_steps = between_starts_solution.front().states.size();
    if (kDebug) {
      std::cout << "Num steps: " << num_steps << std::endl;
    }

    assert(num_steps > 0);

    const std::pair<JointState_t, JointCost_t> first_state =
        getIthFullState(between_starts_solution, 0);
    const JointAction_t first_action(first_state.first.size(), Action::None);
    const JointCost_t first_action_cost(first_state.first.size(), 0);
    nodes_and_came_from.nodes.push_back(Node(
        first_state.first, first_action, first_action_cost, first_state.first,
        utils::sum(first_state.second), first_state.second));

    for (size_t step = 1; step < num_steps; ++step) {
      const std::pair<JointState_t, JointCost_t> current_state =
          getIthFullState(between_starts_solution, step);
      const std::pair<JointAction_t, JointCost_t> current_action =
          getIthFullAction(between_starts_solution, step - 1);

      const std::pair<JointState_t, JointCost_t> previous_state =
          getIthFullState(between_starts_solution, step - 1);

      if (kDebug) {
        std::cout << "Current state: ";
        print(current_state.first);

        std::cout << "Previous state: ";
        print(previous_state.first);

        std::cout << "Current action: ";
        print(current_action.first);
      }
      nodes_and_came_from.nodes.push_back(
          Node(current_state.first, current_action.first, current_action.second,
               previous_state.first, utils::sum(current_state.second),
               current_state.second));
    }

    return nodes_and_came_from;
  }
  
  void dumpOpenSet(const WPS_t* window) {
    std::cout << "vvvvvvvvvvvvvvvvvvvvvvvvvv" << std::endl;
    std::cout << "Looking in open_set: " << std::endl;
    std::vector<Node> node_lst;
    for (const Node& n : window->getSearchState()->open_set) {
      node_lst.push_back(n);
    }
    std::sort(node_lst.begin(), node_lst.end());
    for (const Node& n : node_lst) {
      if (isDebugState(n)) {
        std::cout << n << std::endl;
      }
    }
    std::cout << "Done in open_set: " << std::endl;
    std::cout << "^^^^^^^^^^^^^^^^^^^^^^^^^^" << std::endl;
  }

  bool growAndReplanIn(WPS_t* window, JointPlan_t* solution,
                       TimingGARI* timing_gari) {
    checkClosedSetParentMapDebugState(window->getSearchState()->closed_set, window->getSearchState()->parent_map);
    timing_gari->total_GARI.start();
    static constexpr bool kDebug = true;
    if (kDebug) {
      std::cout << "Grow and replan in" << std::endl;
    }

    timing_gari->time_path_btw_starts.start();

    JointState_t old_starts;
    JointCost_t old_starts_costs;
    JointState_t old_goals;
    JointCost_t old_goals_costs;
    std::tie(old_starts, old_starts_costs, old_goals, old_goals_costs) =
        window->window.getStartsAndGoals(*solution);

    window->window.grow();

    JointState_t new_starts;
    JointCost_t new_starts_costs;
    JointState_t new_goals;
    JointCost_t new_goals_costs;
    std::tie(new_starts, new_starts_costs, new_goals, new_goals_costs) =
        getCollisionFreeStartsGoals(*solution, window);

    if (kDebug) {
      std::cout << "New starts: ";
      print(new_starts);

      std::cout << "Old starts: ";
      print(old_starts);

      std::cout << "Old goals: ";
      print(old_goals);

      std::cout << "New goals: ";
      print(new_goals);
      
      std::cout << "Grow and replan in: " << window->window << '\n';
    }

    verifyStartCostDifference(old_starts_costs, new_starts_costs);
    const NodeAndCameFromValues_t info_between_starts =
        extractPathBetweenStarts(*window, *solution, old_starts,
                                 old_starts_costs, new_starts,
                                 new_starts_costs);

    info_between_starts.verify(*window, old_starts, new_starts);

    timing_gari->time_path_btw_starts.stop();

    if (kDebug) {
      std::cout << "Between Starts Nodes!\n";
      for (const Node& n : info_between_starts.nodes) {
        std::cout << n << std::endl;
      }
      std::cout << "idx " << window->ss_index
                << " growAndReplanIn Num agents: " << new_starts.size()
                << std::endl;
    }

    verifySolutionValid(*solution);
    if (window->getSearchState()->previous_start != old_starts) {
      TimingPlanIn tpi;
      bool pi_res = planIn(window, solution, &tpi);
      verifySolutionValid(*solution);
      timing_gari->total_GARI.stop();
      return pi_res;
    }
    
//     std::cout << "Stage1 before: " << std::endl;
//     print(*solution);
    
    checkClosedSetParentMapDebugState(window->getSearchState()->closed_set, window->getSearchState()->parent_map);
    
    const Cost old_goal_g_score_sum =
        window->getSearchState()->open_set.top().g_score_sum;
    bool s1_res = Stage1(window, old_starts, old_goals, *solution, old_goal_g_score_sum,
           &(timing_gari->timing_stage1));
    if (!s1_res) {
      timing_gari->total_GARI.stop();
      return false;
    }
    
    checkClosedSetParentMapDebugState(window->getSearchState()->closed_set, window->getSearchState()->parent_map);
    
//     dumpOpenSet(window);
    
    verifySolutionValid(*solution);
    std::cout << "Stage1 after: " << std::endl;
    print(*solution);
    bool s2_res = Stage2(window, info_between_starts, new_starts, old_goals, *solution,
           old_goal_g_score_sum, &(timing_gari->timing_stage2));
    if (!s2_res) {
      timing_gari->total_GARI.stop();
      return false;
    }
    
    checkClosedSetParentMapDebugState(window->getSearchState()->closed_set, window->getSearchState()->parent_map);
    
//     dumpOpenSet(window);
    
    verifySolutionValid(*solution);
    verifyOpenSet(*window, *solution);
    std::cout << "Stage2 after: " << std::endl;
    print(*solution);
    bool s3_res = Stage3(window, solution, new_starts, new_starts_costs, new_goals,
           new_goals_costs, old_goals, &(timing_gari->timing_stage3));
    //         print(*solution);
    if (!s3_res) {
      timing_gari->total_GARI.stop();
      return false;
    }
    checkClosedSetParentMapDebugState(window->getSearchState()->closed_set, window->getSearchState()->parent_map);
//     dumpOpenSet(window);
    
//     std::cout << "Stage3 after: " << std::endl;
//     print(*solution);
    verifySolutionValid(*solution);
    timing_gari->total_GARI.stop();
    return true;
  }

  bool shouldQuit(const WPSList_t& windows, const Cost& min_cost,
                  const Cost& current_cost) {
    if (min_cost >= current_cost) {
      return true;
    }
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

  inline void verifyPlanInEndpoints(const JointState_t& start,
                             const JointCost_t& start_cost,
                             const JointState_t& goal,
                             const JointCost_t& goal_cost) const {
    if (kProduction) {
      return;
    }
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
    static constexpr bool kDebug = false;
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

    return std::make_tuple(sg_starts, sg_starts_costs, sg_goals,
                           sg_goals_costs);
  }

  JointPlan_t unwindPath(const JointState_t& starts,
                         const JointCost_t& starts_costs,
                         const Node& unwind_start_node,
                         const ParentMap_t& parent_map) {
    return unwindPath(starts, starts_costs, unwind_start_node.state, unwind_start_node.g_score, parent_map);
  }
  
  JointPlan_t unwindPath(const JointState_t& starts,
                         const JointCost_t& starts_costs,
                         const JointState_t& unwind_start_node_state,
                         const JointCost_t& unwind_start_node_g_score,
                         const ParentMap_t& parent_map) {
    static constexpr bool kDebug = true;
    static constexpr size_t kMaxSteps = 2000;
    if (kDebug) {
      std::cout << "Beginning unwind! Unwind node start state\n";
      print(unwind_start_node_state);
    }
    JointPlan_t window_solution(starts.size());
    size_t unwind_steps = 0;
    auto iter = parent_map.find(toParentMapKey(unwind_start_node_state));
    assert(iter != parent_map.end());
    for (; iter != parent_map.end() && iter->first != toParentMapKey(starts);
         iter = parent_map.find(toParentMapKey(iter->second.previous_state))) {
      if (++unwind_steps > kMaxSteps) {
        std::cout << "Too many unwind steps!\n";
        exit(-1);
      }
      if (kDebug) {
        std::cout << "State: ";
      }
      for (size_t i = 0; i < iter->first.size(); ++i) {
        auto& i_solution = window_solution.at(i);
        const State& s_i = iter->second.current_state.at(i);
        const Action& a_i = iter->second.action_out_of_previous_state.at(i);
        const Cost& c_i = iter->second.cost_out_of_previous_state.at(i);
        const Cost& g_i = iter->second.g_score_previous_state.at(i);

        if (!i_solution.states.empty()) {
          assert(i_solution.states.back().first.time >= s_i.time);
        }

        if (kDebug) {
          std::cout << s_i << ' ' << a_i << ' ';
        }

        i_solution.states.push_back({s_i, g_i});
        i_solution.actions.push_back({a_i, c_i});
      }
      if (kDebug) {
        std::cout << std::endl;
        std::cout << "Previous state: ";
        print(iter->second.previous_state);
      }
    }

    assert(iter != parent_map.end());

    for (size_t i = 0; i < window_solution.size(); ++i) {
      auto& solution = window_solution[i];
      assert(!solution.states.empty());
      if (kDebug) {
        std::cout << "Start State: ";
        for (const auto& s : starts) {
          std::cout << s << ' ';
        }
        std::cout << std::endl;
        std::cout << "First time: " << solution.states.back().first.time
                  << " vs " << starts.at(i).time << std::endl;
      }
      assert(solution.states.back().first.time >= starts.at(i).time);
      solution.states.push_back(
          std::make_pair<>(starts.at(i), starts_costs.at(i)));

      std::reverse(solution.states.begin(), solution.states.end());
      std::reverse(solution.actions.begin(), solution.actions.end());
      solution.cost = unwind_start_node_g_score.at(i);
      solution.fmin = 0;
    }

    if (kDebug) {
    for (const auto& solution : window_solution) {
      std::cout << "Solution:\n";
      for (const auto& s : solution.states) {
        std::cout << s.first << ", " << s.second << std::endl;
      }
    }

      std::cout << "Verify unwind!" << std::endl;
    }
    verifySolutionValid(window_solution);
    return window_solution;
  }

  Node neighborInfoToNode(const JointState_t& goals,
                           const JointState_t& current_state,
                           const JointState_t& neighbor_joint_state,
                           const JointAction_t& neighbor_joint_action,
                           const JointCost_t& neighbor_joint_action_cost,
                           const JointCost_t& neighbor_tenative_gscore) {
    Cost f_score =
          utils::sum(neighbor_tenative_gscore) +
          m_env.admissibleJointHeuristic(neighbor_joint_state, goals);
    return Node(neighbor_joint_state, neighbor_joint_action,
                              neighbor_joint_action_cost, current_state,
                              f_score, neighbor_tenative_gscore);
  }
  
  bool insertStateIntoOpen(const JointState_t& goals,
                           const JointState_t& current_state,
                           const JointState_t& neighbor_joint_state,
                           const JointAction_t& neighbor_joint_action,
                           const JointCost_t& neighbor_joint_action_cost,
                           const JointCost_t& neighbor_tenative_gscore,
                           state_to_heap_t* state_to_heap,
                           open_set_t* open_set) {
    auto it = state_to_heap->find(neighbor_joint_state);
    if (it == state_to_heap->end()) {
      const Node n = neighborInfoToNode(goals, current_state, neighbor_joint_state, neighbor_joint_action, neighbor_joint_action_cost, neighbor_tenative_gscore);
      auto handle = open_set->push(n);
      (*handle).handle = handle;
      state_to_heap->insert(std::make_pair<>(neighbor_joint_state, handle));
    } else {
      auto handle = it->second;
      if (utils::sum(neighbor_tenative_gscore) >=
          utils::sum((*handle).g_score)) {
        return true;
      }
      // Update f and g score.
      const Cost delta =
          utils::diffSum((*handle).g_score, neighbor_tenative_gscore);
      assert(delta > 0);
      (*handle).g_score = neighbor_tenative_gscore;
      (*handle).g_score_sum -= delta;
      (*handle).f_score -= delta;
      (*handle).prev_state = current_state;
      (*handle).action_cost = neighbor_joint_action_cost;
      (*handle).action = neighbor_joint_action;
      open_set->increase(handle);
    }

    return false;
  }

  void verifyStateTimeOrdering(const JointState_t& prev_state,
                               const JointState_t& next_state) {
    if (kProduction) {
      return;
    }
    assert(prev_state.size() == next_state.size());
    for (size_t i = 0; i < prev_state.size(); ++i) {
      assert(prev_state.at(i).time <= next_state.at(i).time);
    }
  }

  bool containsGoalWait(const JointAction_t& ja) {
    for (const auto& a : ja) {
      if (a == Action::GoalWait) {
        return true;
      }
    }
    return false;
  }

  bool containsGoalWait(const JointNeighbor_t& jn) {
    for (const auto& n : jn) {
      if (n.action == Action::GoalWait) {
        return true;
      }
    }
    return false;
  }

  enum ProcessNeighborResult {
    ADDOPEN, INCLOSED, NOTINWINDOW, ISCOLLIDING
  };
  
  friend std::ostream& operator<<(std::ostream & os, ProcessNeighborResult r) {
    switch (r) {
      case ADDOPEN: {os << "ADDOPEN"; return os;}
      case INCLOSED: {os << "INCLOSED"; return os; }
      case NOTINWINDOW: { os << "NOTINWINDOW"; return os; };
      case ISCOLLIDING: {os << "ISCOLLIDING"; return os; };
      default: {os << (int) r; return os; }
    }
  }
  
//   std::ostream& ProcessNeighborResult::operator<<(std::ostream& os) {
//       switch(*this) {
//         case ADDOPEN: { os << "ADDOPEN"; break; }
//         case INCLOSED: { os << "INCLOSED"; break; }
//         case NOTINWINDOW: { os << "NOTINWINDOW"; break; }
//         case ISCOLLIDING: { os << "ISCOLLIDING"; break; }
//       }
//       return os;
//   }
  
  ProcessNeighborResult processNeighbor(const JointState_t& starts, const JointState_t& goals,
                       const Node& current,
                       const JointNeighbor_t& joint_neighbor_info,
                       const bool& is_in_window, const closed_set_t& closed_set,
                       state_to_heap_t* state_to_heap, open_set_t* open_set,
                       out_of_window_t* out_of_window, const bool is_debug_state = false) {
    const JointState_t& current_state = current.state;
    const JointCost_t& current_g_score = current.g_score;

    assert(starts.size() == goals.size());
    assert(current_state.size() == starts.size());
    assert(current_g_score.size() == starts.size());

    JointState_t neighbor_joint_state(starts.size());
    JointAction_t neighbor_joint_action(starts.size());
    JointCost_t neighbor_joint_cost(starts.size());
    JointCost_t neighbor_tenative_gscore = current_g_score;
    assert(joint_neighbor_info.size() == neighbor_joint_state.size());
    for (size_t i = 0; i < neighbor_joint_state.size(); ++i) {
      const auto& e = joint_neighbor_info[i];
      neighbor_joint_state[i] = e.state;
      neighbor_joint_action[i] = e.action;
      neighbor_joint_cost[i] = e.cost;
      neighbor_tenative_gscore[i] += e.cost;
      assert(neighbor_joint_state[i].time >= current_state.at(i).time);
    }

    // All variables initialized.

    if (isColliding(neighbor_joint_state, current_state)) {
      return ISCOLLIDING;
    }

    const Node n = neighborInfoToNode(goals, current_state, neighbor_joint_state,
                            neighbor_joint_action, neighbor_joint_cost,
                            neighbor_tenative_gscore);
    if (is_debug_state) {
      std::cout << "Neighbor: " <<  n << std::endl;
    }
    
    if (!is_in_window) {
      out_of_window->insert({neighbor_joint_state, n});
      return NOTINWINDOW;
    }

    if (inClosedSetAndClosed(closed_set, neighbor_joint_state, n.g_score)) {
//       std::cout << "Closed set cost: ";
//       print(closed_set.find(toParentMapKey(neighbor_joint_state))->second);
//       std::cout << "g-score cost: ";
//       print(n.g_score);
      return INCLOSED;
    }

    if (insertStateIntoOpen(goals, current_state, neighbor_joint_state,
                            neighbor_joint_action, neighbor_joint_cost,
                            neighbor_tenative_gscore, state_to_heap,
                            open_set)) {
      return ADDOPEN;
    }
    return ADDOPEN;
  }

  bool isTooManyIterations(const size_t& iterations, const WPS_t& window) {
    switch (window.window.agent_idxs.size()) {
      case 2: return iterations > 5000;
      case 3: return iterations > 20000;
      case 4: return iterations > 350000;
      case 5: return iterations > 500000;
      default: return (iterations > std::pow(50, window.window.agent_idxs.size()));
    }
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

  JointState_t toParentMapKey(JointState_t v) {
    for (State& s : v) {
      s.time = 0;
    }
    return v;
  }
  
  void insertParentMap(ParentMap_t* parent_map, const Node& n) {
    static constexpr bool kDebug = false;
    if (kDebug) {
      std::cout << "p-map insert: " << n << " inserted: ";
    }

    assert(n.state.size() == n.prev_state.size());
    assert(n.state.size() == n.action.size());
    assert(n.state.size() == n.action_cost.size());
    assert(n.state.size() == n.g_score.size());

    for (size_t i = 0; i < n.g_score.size(); ++i) {
      const State& i_state = n.state.at(i);
      const Cost& i_g_score = n.g_score.at(i);
      const Cost& i_time = n.state.at(i).time;
      const Action& i_action = n.action.at(i);
      if (i_time != i_g_score) {
        if (i_action != Action::GoalWait) {
          std::cout << "i_state: " << i_state << " i_g_score: " << i_g_score
                    << " i_time: " << i_time << " i_action: " << i_action
                    << std::endl;
        }
        assert(i_action == Action::GoalWait);
      }
    }

    ParentValue_t value(n.state, n.prev_state, n.action, n.action_cost, n.g_score);
    const auto insert_result = parent_map->insert({toParentMapKey(n.state), value});
    if (!insert_result.second) {
      if (kDebug) {
        std::cout << "Prev: ";
        print(insert_result.first->second.g_score_previous_state, std::cout, "vs Candidate: ");
        print(n.g_score);
      }
      if (utils::sum(insert_result.first->second.g_score_previous_state) >
          utils::sum(n.g_score)) {
        insert_result.first->second = value;
        if (kDebug) {
          std::cout << "updated" << std::endl;
        }
      } else {
        if (kDebug) {
          std::cout << "left" << std::endl;
          std::cout << "v: ";
          print(insert_result.first->second.previous_state);
          print(insert_result.first->second.action_out_of_previous_state);
          std::cout << "k: ";
          print(insert_result.first->first);
        }
      }
    } else {
      if (kDebug) {        
        std::cout << "inserted. Candidate: ";
        print(n.g_score);
      }
    }
  }

  void eraseParentMap(ParentMap_t* parent_map, const JointState_t& key) {
    static constexpr bool kDebug = false;
    if (kDebug) {
      std::cout << "Erasing old parent ";
      print(key, std::cout, " ");
    }
    const auto result = parent_map->find(toParentMapKey(key));
    if (result == parent_map->end()) {
      return;
    }
    assert(toParentMapKey(key) == result->first);
    if (kDebug) {
      const ParentValue_t& v = result->second;
      print(v.g_score_previous_state, std::cout, " => ");
      print(v.previous_state, std::cout, "\n");
    }
    parent_map->erase(toParentMapKey(key));
  }

  void insertClosedSet(closed_set_t* closed_set, const JointState_t& key, const JointCost_t& value) {
     auto res = closed_set->insert({toParentMapKey(key), value });
     if (res.second) {
       return;
     }
     assert(utils::sum(res.first->second) > utils::sum(value));
     if (utils::sum(res.first->second) > utils::sum(value)) {
       res.first->second = value;
     }
  }
  
  bool inClosedSet(const closed_set_t& closed_set, const JointState_t& key) {
    return closed_set.find(toParentMapKey(key)) != closed_set.end();
  }
  
  bool inClosedSetAndClosed(const closed_set_t& closed_set, const JointState_t& key, const JointCost_t& value) {
    auto it = closed_set.find(toParentMapKey(key));
    if (it == closed_set.end()) {
      return false;
    }
    return utils::sum(it->second) <= utils::sum(value);
  }
  
  void eraseClosedSet(closed_set_t* closed_set, const JointState_t& key) {
    static constexpr bool kDebug = false;
    if (kDebug) {
      std::cout << "Erasing ";
      print(key);
    }
    closed_set->erase(toParentMapKey(key));
  }
  
  bool containsState(const Node& n, const State& s, const size_t idx) {
    if (n.state.size() <= idx) {
      return false;
    }
    return n.state.at(idx) == (s);
  }
  
  bool containsState(const JointNeighbor_t& n, const State& s, const size_t idx) {
    if (n.size() <= idx) {
      return false;
    }
    return n.at(idx).state == (s);
  }

  bool planIn(WPS_t* window, JointPlan_t* solution,
              TimingPlanIn* timing_PlanIn) {
    verifyOpenSet(*window, *solution);
    timing_PlanIn->total_PlanIn.start();
    static constexpr bool kDebug = true;
    assert(!(window->window.agent_idxs.empty()));

    JointState_t starts;
    JointCost_t starts_costs;
    JointState_t goals;
    JointCost_t goals_costs;
    std::tie(starts, starts_costs, goals, goals_costs) =
        getCollisionFreeStartsGoals(*solution, window);
    if (kDebug) {
      std::cout << "Plan in: " << window->window << '\n';
      std::cout << "planIn num agents: " << starts.size() << std::endl;
      std::cout << "idx " << window->ss_index << " starts: ";
      print(starts, std::cout, "");
      std::cout << "goals: ";
      print(goals);
    }

    auto* ss = window->getSearchState();
    open_set_t& open_set = ss->open_set;
    state_to_heap_t& state_to_heap = ss->state_to_heap;
    closed_set_t& closed_set = ss->closed_set;
    ParentMap_t& parent_map = ss->parent_map;
    out_of_window_t& out_of_window = ss->out_of_window;
    goal_wait_nodes_t& goal_wait_nodes = ss->goal_wait_nodes;

    open_set.clear();
    state_to_heap.clear();
    closed_set.clear();
    parent_map.clear();
    out_of_window.clear();
    goal_wait_nodes.clear();

    ss->previous_start = starts;

    const Node initial_start_node(
        starts, JointAction_t(starts.size(), Action::None),
        JointCost_t(starts.size(), 0), starts,
        m_env.admissibleJointHeuristic(starts, goals), starts_costs);
    if (kDebug) {
      std::cout << "Initial start node g_value: ";
      print(initial_start_node.g_score);
    }
    auto handle = open_set.push(initial_start_node);
    state_to_heap.insert(std::make_pair<>(starts, handle));
    (*handle).handle = handle;

    timing_PlanIn->timing_expansions.total_S3Exp.start();
    for (size_t expand_count = 0; !open_set.empty(); ++expand_count) {
      timing_PlanIn->num_until_goal_expanded++;
      if (kDebug && expand_count % 10000 == 0) {
        std::cout << "Expanded " << expand_count << "\n";
      }
      if (isTooManyIterations(expand_count, *window)) {
        timing_PlanIn->total_PlanIn.stop();
        return false;
      }

      Node current = open_set.top();

      m_env.onExpandNode(current.state, current.f_score, current.g_score);

      if (m_env.isJointSolution(current.state, goals)) {
        if (kDebug) {
          std::cout << "Goal node has fvalue of " << current.f_score << std::endl;
        }
        insertParentMap(&parent_map, current);
        insertClosedSet(&closed_set, current.state, current.g_score);
        verifyParentMap(parent_map, starts);
        auto window_solution =
            unwindPath(starts, starts_costs, current, parent_map);
        const auto min_max = getMinMaxCostFromPlan(window_solution);
        for (const auto& e : starts_costs) {
          assert(min_max.first == e);
        }
        ss->min_cost = min_max.first;
        ss->max_cost = min_max.second;
        timing_PlanIn->timing_expansions.total_S3Exp.stop();
        timing_PlanIn->total_PlanIn.stop();
        verifyOpenSet(*window, *solution);
        return insertWindowPath(&window_solution, window->window, starts_costs,
                                goals_costs, solution);
      }

      timing_PlanIn->timing_expansions.time_top_o.start();
      open_set.pop();
      state_to_heap.erase(current.state);
      timing_PlanIn->timing_expansions.time_top_o.stop();
      timing_PlanIn->timing_expansions.time_check_in_c.start();
      if (inClosedSetAndClosed(closed_set, current.state, current.g_score)) {
        timing_PlanIn->timing_expansions.time_check_in_c.stop();
        continue;
      }
      insertClosedSet(&closed_set, current.state, current.g_score);
      timing_PlanIn->timing_expansions.time_check_in_c.stop();

      timing_PlanIn->timing_expansions.time_add_parent.start();
      insertParentMap(&parent_map, current);
      timing_PlanIn->timing_expansions.time_add_parent.stop();

      auto neighbor_generator = m_env.getJointWindowNeighbors(
          current.state, current.action, goals, window->window, *solution);

      JointNeighbor_t joint_neighbor_info(current.state.size());
      bool joint_neighbor_in_window = true;
      bool neighbor_has_goal_wait = false;
      while (!neighbor_generator.atEnd()) {
        timing_PlanIn->timing_expansions.num_neighbors++;
        timing_PlanIn->timing_expansions.time_add_neighbors_to_o.start();
        const auto& generator_output = neighbor_generator.getAndIncrement();
        extractJointNeighborInfo(&joint_neighbor_info,
                                 &joint_neighbor_in_window, generator_output);
                
        neighbor_has_goal_wait =
            neighbor_has_goal_wait || containsGoalWait(joint_neighbor_info);
        for (size_t i = 0; i < current.action.size(); ++i) {
          const auto& n = joint_neighbor_info.at(i);
          if (current.action.at(i) == Action::GoalWait) {
            assert(n.action == Action::GoalWait);
          }
        }

        processNeighbor(starts, goals, current, joint_neighbor_info,
                        joint_neighbor_in_window, closed_set, &state_to_heap,
                        &open_set, &out_of_window);
        
        timing_PlanIn->timing_expansions.time_add_neighbors_to_o.stop();
      }

      if (neighbor_has_goal_wait) {
        goal_wait_nodes.push_back(current);
      }
    }
    timing_PlanIn->timing_expansions.total_S3Exp.stop();
    timing_PlanIn->total_PlanIn.stop();
    return false;
  }

  bool insertWindowPath(JointPlan_t* window_path, const Window& window,
                        const JointCost_t& starts_cost,
                        const JointCost_t& goals_cost, JointPlan_t* full_path) {
    static constexpr bool kDebug = false;
    if (kDebug) {
      std::cout << "===============insertWindowPath\n";
    }
    assert(starts_cost.size() == goals_cost.size());
    assert(window.agent_idxs.size() == goals_cost.size());

    if (kDebug) {
      std::cout << "Full Before:\n";
      print(*full_path);

      std::cout << "Window path before trim:\n";
      print(*window_path);
    }

    for (size_t i = 0; i < window.agent_idxs.size(); ++i) {
      auto& individual_window_plan = window_path->at(i);

      size_t ignore_wait_command_count = 0;
      for (auto it = individual_window_plan.actions.rbegin();
           it != individual_window_plan.actions.rend(); ++it) {
        if (it->second == 0 || it->first == Action::GoalWait) {
          ignore_wait_command_count++;
        } else {
          break;
        }
      }
      assert(ignore_wait_command_count <=
             individual_window_plan.actions.size());

      individual_window_plan.states.resize(
          individual_window_plan.states.size() - ignore_wait_command_count);
      individual_window_plan.actions.resize(
          individual_window_plan.actions.size() - ignore_wait_command_count);
    }

    if (kDebug) {
      std::cout << "Window path after trim: " << std::endl;
      print(*window_path);
    }

    for (size_t i = 0; i < window.agent_idxs.size(); ++i) {
      const auto& agent_idx = window.agent_idxs.at(i);
      auto& individual_initial_plan = full_path->at(agent_idx);
      auto& individual_window_plan = window_path->at(i);

      assert(individual_window_plan.states.size() ==
             individual_window_plan.actions.size() + 1);

      const Cost initial_start_idx = starts_cost.at(i);
      const Cost initial_goal_idx = goals_cost.at(i);
      if (kDebug) {
        std::cout << "initial_start_idx: " << initial_start_idx
                  << " initial_goal_idx: " << initial_goal_idx << std::endl;
      }

      assert(individual_initial_plan.states.at(initial_start_idx).first ==
             individual_window_plan.states.front().first);
      assert(individual_initial_plan.states.at(initial_goal_idx)
                 .first.equalExceptTime(
                     individual_window_plan.states.back().first));

      const Cost cost_delta =
          individual_window_plan.states.back().second - initial_goal_idx;

      if (kDebug) {
        std::cout << "Cost delta: " << cost_delta << std::endl;
      }

      for (auto it =
               individual_initial_plan.states.begin() + initial_goal_idx + 1;
           it != individual_initial_plan.states.end(); ++it) {
        it->second += cost_delta;
        it->first.time += cost_delta;
      }

      //       std::cout << "Ignoring last " << ignore_wait_command_count
      //                 << " commands\n";

      individual_initial_plan.states.erase(
          individual_initial_plan.states.begin() + initial_start_idx,
          individual_initial_plan.states.begin() + initial_goal_idx + 1);
      individual_initial_plan.states.insert(
          individual_initial_plan.states.begin() + initial_start_idx,
          individual_window_plan.states.begin(),
          individual_window_plan.states.end());

      individual_initial_plan.actions.erase(
          individual_initial_plan.actions.begin() + initial_start_idx,
          individual_initial_plan.actions.begin() + initial_goal_idx);
      individual_initial_plan.actions.insert(
          individual_initial_plan.actions.begin() + initial_start_idx,
          individual_window_plan.actions.begin(),
          individual_window_plan.actions.end());
      
      individual_initial_plan.cost = individual_initial_plan.states.back().second;
    }

    if (kDebug) {
      std::cout << "Full After:\n";
      print(*full_path);
    }

    return true;
  }

  bool planIndividually(const JointState_t& initial_states,
                        JointPlan_t& solution, TimingWAMPF* timing) {
    timing->num_agents = initial_states.size();
    solution.resize(initial_states.size());
    timing->time_individual_plan.start();
    for (size_t i = 0; i < initial_states.size(); ++i) {
      LowLevelEnvironment llenv(m_env, i);
      LowLevelSearch_t lowLevel(llenv);
      if (!lowLevel.search(initial_states[i], solution[i])) {
        timing->time_individual_plan.stop();
        return false;
      }
    }
    timing->time_individual_plan.stop();
    return true;
  }

  Cost getSolutionCost(const JointPlan_t& solution) {
    Cost total_cost = 0;
    for (const IndividualPlan_t& p : solution) {
      assert(!p.states.empty());
      assert(p.states.back().second >= 0);
      total_cost += p.cost;
    }
    return total_cost;
  }

  void SaveTimingData(const TimingWAMPF& timing, const bool is_final,
                      const std::string& save_infix) const {
    size_t additive = (is_final ? 1 : 0);
    std::string filename = "iteration_" + save_infix + "_" +
                           std::to_string(timing.num_recWAMPF + additive) +
                           ".timing";
    std::ofstream f(filename);
    f << timing << "\n";
    f.close();
  }
  
  void PrintSearchStateOpen(const State& s, size_t idx, const WPS_t& window) {
    const open_set_t& open_set = window.getSearchState()->open_set;
    std::vector<Node> nodes;
    for (const Node& n : open_set) {
      if (n.state.at(idx).equalExceptTime(s)) {
        nodes.push_back(n);
      }
    }
    std::sort(nodes.begin(), nodes.end());
    std::reverse(nodes.begin(), nodes.end());
    if (nodes.size() > 10) {
      nodes.erase(nodes.begin() + 10, nodes.end());
    }
    for (const Node& n : nodes) {
      std::cout << n << std::endl;
    }
  }
  
    void PrintSearchStateOutOfWindow(const State& s, size_t idx, const WPS_t& window) {
    const out_of_window_t& out_of_window = window.getSearchState()->out_of_window;
    std::vector<Node> nodes;
    for (const std::pair<JointState_t, Node>& n : out_of_window) {
      if (n.second.state.at(idx).equalExceptTime(s)) {
        nodes.push_back(n.second);
      }
    }
    std::sort(nodes.begin(), nodes.end());
    std::reverse(nodes.begin(), nodes.end());
    if (nodes.size() > 10) {
      nodes.erase(nodes.begin() + 10, nodes.end());
    }
    for (const Node& n : nodes) {
      std::cout << n << std::endl;
    }
  }
  
  void PrintSearchStateClosed(const State& s, size_t idx, const WPS_t& window) {
    const closed_set_t& closed_set = window.getSearchState()->closed_set;
    std::vector<std::pair<JointState_t, JointCost_t>> nodes;
    for (const std::pair<JointState_t, JointCost_t>& n : closed_set) {
      if (n.first.at(idx).equalExceptTime(s)) {
        nodes.push_back(n);
      }
    }
    std::sort(nodes.begin(), nodes.end(), [] (const auto& e1, const auto& e2) -> bool { return utils::sum(e1.second) < utils::sum(e2.second); });
    //std::reverse(nodes.begin(), nodes.end());
    if (nodes.size() > 10) {
      nodes.erase(nodes.begin() + 10, nodes.end());
    }
    for (const std::pair<JointState_t, JointCost_t>& n : nodes) {
      std::cout << "JointState: ";
      print(n.first, std::cout, " JointCost ");
      print(n.second);
    }
  }

 public:
  XStar(Environment& environment) : m_env(environment) {}

  bool search(const JointState_t& initial_starts, 
              const JointState_t& final_goal, 
              JointPlan_t& solution,
              const std::string& save_infix) {
    TimingWAMPF timing;
    timing.total_WAMPF.start();
    if (!planIndividually(initial_starts, solution, &timing)) {
      timing.total_WAMPF.stop();
      return false;
    }

    const Cost optimal_solution_lower_bound = getSolutionCost(solution);
    Cost solution_cost = std::numeric_limits<Cost>::max();
    WPSList_t windows;
    SSList_t search_states;
    timing.total_WAMPF.stop();
    do {
      timing.total_WAMPF.start();
      timing.num_recWAMPF++;
      if (timing.num_recWAMPF == 1) {
        timing.time_first_plan.start();
      }
      recWAMPF(initial_starts, final_goal, &windows, &search_states, &solution, &timing.timing_recWAMPF);
      solution_cost = getSolutionCost(solution);
      timing.optimality_bound =
          static_cast<float>(solution_cost) /
          static_cast<float>(optimal_solution_lower_bound);
      timing.successive_bounds.push_back(timing.optimality_bound);
      timing.total_WAMPF.stop();
      timing.successive_runtimes.push_back(timing.total_WAMPF.get());
      timing.total_WAMPF.start();      
      if (timing.num_recWAMPF == 1) {
        for (const WPS_t& w : windows) {
          timing.num_max_agents_in_window_first_iteration =
              std::max(timing.num_max_agents_in_window_first_iteration,
                       w.window.agent_idxs.size());
        }
        timing.time_first_plan.stop();
      }
      timing.total_WAMPF.stop();
      SaveTimingData(timing, false, save_infix);
    } while (!shouldQuit(windows, optimal_solution_lower_bound, solution_cost));
    timing.is_optimal = true;
    timing.optimality_bound = 1.0;
    timing.successive_bounds.push_back(timing.optimality_bound);
    SaveTimingData(timing, true, save_infix);
    return true;
  }
};  // namespace libMultiRobotPlanning

}  // namespace libMultiRobotPlanning
