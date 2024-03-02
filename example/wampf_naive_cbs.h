#pragma once

#include <algorithm>
#include <vector>

#include <libMultiRobotPlanning/cbs.hpp>
#include <libMultiRobotPlanning/wampf_utils.hpp>

#include "wampf_individual.h"
#include "wampf_naive_cbs_env.h"
#include "wampf_state.h"
#include "wampf_window.h"

namespace wampf_impl {

using libMultiRobotPlanning::CBS;
using libMultiRobotPlanning::IndividualSpaceAction;
using libMultiRobotPlanning::IndividualSpaceAStar;
using libMultiRobotPlanning::IndividualSpaceEnvironment;
using libMultiRobotPlanning::Neighbor;
using libMultiRobotPlanning::PlanResult;
using libMultiRobotPlanning::State;
using Window = libMultiRobotPlanning::Window<>;

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
  JointPath* path_;

 public:
  NWAStarGridWAMPFImplementation(size_t dimx, size_t dimy,
                                 std::unordered_set<State> obstacles,
                                 const JointState& start,
                                 const JointState& goal, JointPath* path)
      : dimx_(dimx),
        dimy_(dimy),
        obstacles_(std::move(obstacles)),
        start_(start),
        goal_(goal),
        path_(path) {}

  std::optional<Window> FirstCollisionWindow() {
    auto res = FirstConflict(); // return where is the first conflict (res->first) and the agents involved (res->second)
    if (!res) {
      return {};
    }
    return {{std::move(res->first), std::move(res->second)}};
  }

  std::pair<int, int> GetStartEndIndices(const Window& w) const {
    // Start and goal indices in the order of the index list contained in the
    // window.
    const auto start_goal_idxs =
        libMultiRobotPlanning::wampf::GetWindowStartGoalIndexes(*path_, w);
    NP_CHECK(!start_goal_idxs.empty());
    NP_CHECK_GE(start_goal_idxs.size(), 2);

    int min_start_idx = start_goal_idxs.front().first;
    int max_goal_idx = start_goal_idxs.front().second;
    for (const auto& start_goal : start_goal_idxs) {
      min_start_idx = std::min(min_start_idx, start_goal.first);
      max_goal_idx = std::max(max_goal_idx, start_goal.second);
    }

    // Verify that each start index is within the range of the path.
    for (const auto& idx : w.agent_idxs_) {
      NP_CHECK(idx < path_->size());
      NP_CHECK_LT(min_start_idx, static_cast<int>((*path_)[idx].states.size()));
    }
    return {min_start_idx, max_goal_idx};
  }

  void PlanIn(Window* w) {
    auto [start_idx, goal_idx] = GetStartEndIndices(*w); // start e goal are the time indexes of the window

    std::vector<naive_cbs_wampf_impl::CBSState> start_state;
    JointState goal_state;
    for (const auto& idx : w->agent_idxs_) {
      const auto start = GetState(idx, start_idx);
      start_state.push_back(
          naive_cbs_wampf_impl::CBSState(0, start.x, start.y));
      goal_state.emplace_back(GetState(idx, goal_idx));
    }

    naive_cbs_wampf_impl::NaiveCBSEnvironment<State> mapf(
        dimx_, dimy_, obstacles_, goal_state);
    CBS<naive_cbs_wampf_impl::CBSState, naive_cbs_wampf_impl::CBSAction, int,
        naive_cbs_wampf_impl::Conflict, naive_cbs_wampf_impl::Constraints,
        naive_cbs_wampf_impl::NaiveCBSEnvironment<State>>
        cbs(mapf);
    std::vector<PlanResult<naive_cbs_wampf_impl::CBSState,
                           naive_cbs_wampf_impl::CBSAction, int>>
        cbs_repair;
    bool success = cbs.search(start_state, cbs_repair); // Use of CBS for path search
    if (!success) {
      w->Grow();
      PlanIn(w);
    }

    const auto repair = CBSRepairToStandardRepair(cbs_repair);

    for (size_t i = 0; i < repair.size(); ++i) {
      NP_CHECK(i < w->agent_idxs_.size());
      const size_t path_idx = w->agent_idxs_[i];
      NP_CHECK(path_idx < path_->size());
      auto& local_path = (*path_)[path_idx];
     
      // Check if goal_idx is greater than the last timestamp of the local path.
      const auto local_goal_idx = local_path.states.back().second;
      if (goal_idx > local_goal_idx) {
        local_path = libMultiRobotPlanning::wampf::InsertPathRepair(
          local_path, repair[i], start_idx, local_goal_idx);  
      }
      else{
        local_path = libMultiRobotPlanning::wampf::InsertPathRepair(
          local_path, repair[i], start_idx, goal_idx);
      }
    }
  }

  void GrowAndReplanIn(Window* w) { w->Grow(); }

 private:
  IndividualSpaceAction CBSActionToIndividualSpaceAction(
      const naive_cbs_wampf_impl::CBSAction& a) const {
    switch (a) {
      case naive_cbs_wampf_impl::CBSAction::Up:
        return IndividualSpaceAction::Up;
      case naive_cbs_wampf_impl::CBSAction::Down:
        return IndividualSpaceAction::Down;
      case naive_cbs_wampf_impl::CBSAction::Left:
        return IndividualSpaceAction::Left;
      case naive_cbs_wampf_impl::CBSAction::Right:
        return IndividualSpaceAction::Right;
      case naive_cbs_wampf_impl::CBSAction::Wait:
        return IndividualSpaceAction::Wait;
    }
    return IndividualSpaceAction::Wait;
  }

  std::vector<PlanResult<State, IndividualSpaceAction, int>>
  CBSRepairToStandardRepair(
      const std::vector<PlanResult<naive_cbs_wampf_impl::CBSState,
                                   naive_cbs_wampf_impl::CBSAction, int>>&
          cbs_repair) {
    std::vector<PlanResult<State, IndividualSpaceAction, int>> repair;
    for (const auto& r : cbs_repair) {
      repair.push_back({});
      for (const auto& s : r.states) {
        repair.back().states.push_back({{s.first.x, s.first.y}, s.second});
      }
      for (const auto& a : r.actions) {
        repair.back().actions.push_back(
            {CBSActionToIndividualSpaceAction(a.first), a.second});
      }
      repair.back().cost = r.cost;
      repair.back().fmin = r.fmin;
    }
    return repair;
  }

  State GetState(size_t agent_idx, Cost timestep) const {
    NP_CHECK(agent_idx < (*path_).size());
    const auto& agent_path = (*path_)[agent_idx];
    if (timestep < static_cast<int>(agent_path.states.size())) {
      NP_CHECK(agent_path.states[timestep].second == timestep);
      return agent_path.states[timestep].first;
    }
    NP_CHECK(!agent_path.states.empty());
    return agent_path.states.back().first;
  }

  std::optional<std::pair<State, std::vector<size_t>>> FirstConflict() {
    int max_t = 0;
    for (const auto& sol : (*path_)) {
      max_t = std::max<int>(max_t, sol.states.size() - 1);
    }

    for (int t = 0; t < max_t; ++t) {
      // check drive-drive vertex collisions
      for (size_t agent_i = 0; agent_i < (*path_).size(); ++agent_i) {
        State state_i = GetState(agent_i, t);
        for (size_t agent_j = agent_i + 1; agent_j < (*path_).size();
             ++agent_j) {
          State state_j = GetState(agent_j, t);
          if (state_i == state_j) {
            return {{state_i, {agent_i, agent_j}}};
          }
        }
      }
      // drive-drive edge (swap)
      for (size_t agent_i = 0; agent_i < (*path_).size(); ++agent_i) {
        State state_i_t = GetState(agent_i, t);
        State state_i_tp1 = GetState(agent_i, t + 1);
        for (size_t agent_j = agent_i + 1; agent_j < (*path_).size();
             ++agent_j) {
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

}  // namespace wampf_impl
