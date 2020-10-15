#include <fstream>
#include <iostream>
#include <optional>
#include <unordered_set>

#include <gtest/gtest.h>

#include <libMultiRobotPlanning/wampf_utils.hpp>

#include "wampf_individual.h"
#include "wampf_state.h"

using libMultiRobotPlanning::IndividualSpaceAction;
using libMultiRobotPlanning::PlanResult;
using libMultiRobotPlanning::State;
using libMultiRobotPlanning::wampf::InsertPathRepair;
using PR = PlanResult<State, IndividualSpaceAction, int>;
using Action = IndividualSpaceAction;

void VerifyPathIntegrity(const PR& result) {
  EXPECT_EQ(result.actions.size() + 1, result.states.size());
  EXPECT_EQ(result.fmin, result.cost);
  for (size_t i = 1; i < result.states.size(); ++i) {
    const auto state_i_tuple = result.states[i];
    const auto state_im1_tuple = result.states[i - 1];
    EXPECT_EQ(1, state_i_tuple.second - state_im1_tuple.second);
    const auto state_i = state_i_tuple.first;
    const auto state_im1 = state_im1_tuple.first;
    EXPECT_LT(i - 1, result.actions.size());
    const auto action = result.actions[i - 1].first;
    switch (action) {
      case Action::Up: {
        EXPECT_EQ(state_im1.x, state_i.x);
        EXPECT_EQ(state_im1.y + 1, state_i.y);
        break;
      }
      case Action::Down: {
        EXPECT_EQ(state_im1.x, state_i.x);
        EXPECT_EQ(state_im1.y - 1, state_i.y);
        break;
      }
      case Action::Left: {
        EXPECT_EQ(state_im1.x - 1, state_i.x);
        EXPECT_EQ(state_im1.y, state_i.y);
        break;
      }
      case Action::Right: {
        EXPECT_EQ(state_im1.x + 1, state_i.x);
        EXPECT_EQ(state_im1.y, state_i.y);
        break;
      }
    }
  }
}

TEST(InsertLongerRepair, StraightToUpDown) {
  PR full_path;
  full_path.states = {{{0, 0}, 0}, {{0, 1}, 1}, {{0, 2}, 2},
                      {{0, 3}, 3}, {{0, 4}, 4}, {{0, 5}, 5}};
  full_path.actions = {{Action::Up, 1},
                       {Action::Up, 1},
                       {Action::Up, 1},
                       {Action::Up, 1},
                       {Action::Up, 1}};
  full_path.cost = 5;
  full_path.fmin = 5;

  PR repair;
  repair.states = {{{0, 1}, 0}, {{1, 1}, 1}, {{1, 2}, 2}, {{0, 2}, 3}};
  repair.actions = {{Action::Right, 1}, {Action::Up, 1}, {Action::Left, 1}};
  repair.cost = 3;
  repair.fmin = 3;

  int repair_start = 1;
  int repair_end = 2;

  PR result = InsertPathRepair(full_path, repair, repair_start, repair_end);
  EXPECT_EQ(result.cost, 7);
  VerifyPathIntegrity(result);
}

TEST(InsertShorterRepair, UpDownToStraight) {
  PR full_path;
  full_path.states = {{{0, 0}, 0}, {{0, 1}, 1}, {{1, 1}, 2},
                      {{1, 2}, 3}, {{0, 2}, 4}, {{0, 3}, 5}};
  full_path.actions = {{Action::Up, 1},
                       {Action::Right, 1},
                       {Action::Up, 1},
                       {Action::Left, 1},
                       {Action::Up, 1}};
  full_path.cost = 5;
  full_path.fmin = 5;

  PR repair;
  repair.states = {{{0, 1}, 0}, {{0, 2}, 1}};
  repair.actions = {{Action::Up, 1}};
  repair.cost = 1;
  repair.fmin = 1;

  int repair_start = 1;
  int repair_end = 4;

  PR result = InsertPathRepair(full_path, repair, repair_start, repair_end);
  EXPECT_EQ(result.cost, 3);
  VerifyPathIntegrity(result);
}

TEST(InsertShorterRepair, UpDownToStraight2) {
  PR full_path;
  full_path.states = {{{0, 0}, 0}, {{0, 1}, 1}, {{0, 2}, 2}, {{1, 2}, 3},
                      {{1, 3}, 4}, {{0, 3}, 5}, {{0, 4}, 6}};
  full_path.actions = {{Action::Up, 1}, {Action::Up, 1},   {Action::Right, 1},
                       {Action::Up, 1}, {Action::Left, 1}, {Action::Up, 1}};
  full_path.cost = 6;
  full_path.fmin = 6;

  PR repair;
  repair.states = {{{0, 2}, 0}, {{0, 3}, 1}};
  repair.actions = {{Action::Up, 1}};
  repair.cost = 1;
  repair.fmin = 1;

  int repair_start = 2;
  int repair_end = 5;

  PR result = InsertPathRepair(full_path, repair, repair_start, repair_end);
  EXPECT_EQ(result.cost, 4);
  VerifyPathIntegrity(result);
}