#include <fstream>
#include <iostream>
#include <optional>
#include <unordered_set>

#include <libMultiRobotPlanning/individual_space_astar.hpp>
#include <libMultiRobotPlanning/neighbor.hpp>
#include <libMultiRobotPlanning/wampf.hpp>

#include <boost/functional/hash.hpp>
#include <boost/program_options.hpp>

#include "wampf_individual.h"
#include "wampf_state.h"
#include "wampf_window.h"

using libMultiRobotPlanning::IndividualSpaceAction;
using libMultiRobotPlanning::IndividualSpaceAStar;
using libMultiRobotPlanning::IndividualSpaceEnvironment;
using libMultiRobotPlanning::Neighbor;
using libMultiRobotPlanning::PlanResult;
using libMultiRobotPlanning::State;
using libMultiRobotPlanning::Window;

using Cost = int;
using JointState = std::vector<State>;
using JointPath = std::vector<PlanResult<State, IndividualSpaceAction, Cost>>;

class NWAStarGridWAMPFImplementation {
 private:
  size_t dimx_;
  size_t dimy_;
  std::unordered_set<State> obstacles_;
  JointState start_;
  JointState goal_;
  const JointPath& path_;

 public:
  NWAStarGridWAMPFImplementation(size_t dimx, size_t dimy,
                                 std::unordered_set<State> obstacles,
                                 const JointState& start,
                                 const JointState& goal, const JointPath& path)
      : dimx_(dimx),
        dimy_(dimy),
        obstacles_(std::move(obstacles)),
        start_(start),
        goal_(goal),
        path_(path) {}

  std::optional<Window> FirstCollisionWindow() {
    auto res = FirstConflict();
    if (!res) {
      return {};
    }
    return {{res->first, res->second}};
  }

  void PlanIn(Window* w) {}

  void GrowAndReplanIn(Window* w) {}

 private:
  State GetState(size_t agent_idx, Cost timestep) const {
    NP_CHECK(agent_idx < path_.size());
    const auto& agent_path = path_[agent_idx];
    if (timestep < static_cast<int>(agent_path.states.size())) {
      NP_CHECK(agent_path.states[timestep].second == timestep);
      return agent_path.states[timestep].first;
    }
    NP_CHECK(!agent_path.states.empty());
    return agent_path.states.back().first;
  }

  std::optional<std::pair<State, std::vector<size_t>>> FirstConflict() {
    int max_t = 0;
    for (const auto& sol : path_) {
      max_t = std::max<int>(max_t, sol.states.size() - 1);
    }

    for (int t = 0; t < max_t; ++t) {
      // check drive-drive vertex collisions
      for (size_t agent_i = 0; agent_i < path_.size(); ++agent_i) {
        State state_i = GetState(agent_i, t);
        for (size_t agent_j = agent_i + 1; agent_j < path_.size(); ++agent_j) {
          State state_j = GetState(agent_j, t);
          if (state_i == state_j) {
            return {{state_i, {agent_i, agent_j}}};
          }
        }
      }
      // drive-drive edge (swap)
      for (size_t agent_i = 0; agent_i < path_.size(); ++agent_i) {
        State state_i_t = GetState(agent_i, t);
        State state_i_tp1 = GetState(agent_i, t + 1);
        for (size_t agent_j = agent_i + 1; agent_j < path_.size(); ++agent_j) {
          State state_j_t = GetState(agent_j, t);
          State state_j_tp1 = GetState(agent_j, t + 1);
          if (state_i_t == state_j_tp1 && state_i_tp1 == state_j_t) {
            return {{state_i_tp1, {agent_i, agent_j}}};
          }
        }
      }
    }
    return {};
  }
};

int main() {
  using WAMPF = libMultiRobotPlanning::wampf::WAMPF<
      State, IndividualSpaceAction, Cost, Window,
      IndividualSpaceAStar<State, IndividualSpaceAction, Cost,
                           IndividualSpaceEnvironment>,
      NWAStarGridWAMPFImplementation>;

  std::cout << "Starting WAMPF!" << std::endl;
  JointState start_state = {{0, 0}, {0, 1}};
  JointState goal_state = {{5, 0}, {5, 1}};
  WAMPF wampf(100, 100, {}, start_state, goal_state);
  wampf.RecWAMPF();
}