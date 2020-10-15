#pragma once

#include <algorithm>
#include <vector>

#include "wampf_state.h"

namespace libMultiRobotPlanning {

struct Window {
 private:
  Window() = delete;

  State min_pos_;
  State max_pos_;
  std::vector<size_t> agent_idxs_;

  static constexpr int kStartRadius = 2;
  static constexpr int kRadiusGrowth = 1;

 public:
  Window(State state, const std::vector<size_t> agent_idxs)
      : min_pos_(state), max_pos_(state), agent_idxs_(agent_idxs) {
    min_pos_.x -= kStartRadius;
    min_pos_.y -= kStartRadius;
    max_pos_.x += kStartRadius;
    max_pos_.y += kStartRadius;
  }

  bool operator==(const Window& other) const {
    return (min_pos_ == other.min_pos_) && (max_pos_ == other.max_pos_) &&
           (agent_idxs_ == other.agent_idxs_);
  }

  bool operator!=(const Window& other) const { return !(*this == other); }

  Window(State min_pos, State max_pos, const std::vector<size_t> agent_idxs)
      : min_pos_(min_pos), max_pos_(max_pos), agent_idxs_(agent_idxs) {}

  bool HasAgent(const size_t agent_idx) const {
    return (std::find(agent_idxs_.begin(), agent_idxs_.end(), agent_idx) !=
            agent_idxs_.end());
  }

  bool Contains(const State& s) const {
    return ((s.x >= min_pos_.x && s.x <= max_pos_.x) &&
            (s.y >= min_pos_.y && s.y <= max_pos_.y));
  }

  bool OverlappingAgents(const Window& other) const {
    for (const auto& a : other.agent_idxs_) {
      if (HasAgent(a)) {
        return true;
      }
    }
    return false;
  }

  bool Overlaps(const Window& other) const {
    if (!OverlappingAgents(other)) {
      return false;
    }

    State off1(min_pos_.x, max_pos_.y);
    State off2(max_pos_.x, min_pos_.y);

    // Check if our four corners are inside their box.
    if (other.Contains(min_pos_) || other.Contains(max_pos_) ||
        other.Contains(off1) || other.Contains(off2)) {
      return true;
    }

    State other_off1(other.min_pos_.x, other.max_pos_.y);
    State other_off2(other.max_pos_.x, other.min_pos_.y);

    // Check if their four corners are inside our box.
    if (Contains(other.min_pos_) || Contains(other.max_pos_) ||
        Contains(other_off1) || Contains(other_off2)) {
      return true;
    }
    return false;
  }

  bool SuccessorOverlaps(const Window& other) const {
    if (!OverlappingAgents(other)) {
      return false;
    }

    State min_growth(min_pos_.x - kRadiusGrowth, min_pos_.y - kRadiusGrowth);
    State max_growth(max_pos_.x + kRadiusGrowth, max_pos_.y + kRadiusGrowth);
    State off1(min_pos_.x - kRadiusGrowth, max_pos_.y + kRadiusGrowth);
    State off2(max_pos_.x + kRadiusGrowth, min_pos_.y - kRadiusGrowth);

    // Check if our four corners are inside their box.
    if (other.Contains(min_growth) || other.Contains(max_growth) ||
        other.Contains(off1) || other.Contains(off2)) {
      return true;
    }

    // Inflating their box is equivalent to inflating our box.
    State other_min_growth(other.min_pos_.x - kRadiusGrowth,
                           other.min_pos_.y - kRadiusGrowth);
    State other_max_growth(other.max_pos_.x + kRadiusGrowth,
                           other.max_pos_.y + kRadiusGrowth);
    State other_off1(other.min_pos_.x - kRadiusGrowth,
                     other.max_pos_.y + kRadiusGrowth);
    State other_off2(other.max_pos_.x + kRadiusGrowth,
                     other.min_pos_.y - kRadiusGrowth);

    // Check if their four corners are inside our box.
    if (Contains(other.min_pos_) || Contains(other.max_pos_) ||
        Contains(other_off1) || Contains(other_off2)) {
      return true;
    }
    return false;
  }

  bool ShouldQuit() const { return false; }

  Window Merge(const Window& o) const {
    int min_x = std::min(min_pos_.x, o.min_pos_.x);
    int max_x = std::max(max_pos_.x, o.max_pos_.x);
    int min_y = std::min(min_pos_.y, o.min_pos_.y);
    int max_y = std::max(max_pos_.y, o.max_pos_.y);

    auto joined_agent_idxs = agent_idxs_;
    joined_agent_idxs.insert(joined_agent_idxs.end(), o.agent_idxs_.begin(),
                             o.agent_idxs_.end());
    std::sort(joined_agent_idxs.begin(), joined_agent_idxs.end());
    const auto it =
        std::unique(joined_agent_idxs.begin(), joined_agent_idxs.end());
    joined_agent_idxs.resize(std::distance(joined_agent_idxs.begin(), it));
    std::sort(joined_agent_idxs.begin(), joined_agent_idxs.end());
    return {{min_x, min_y}, {max_x, max_y}, joined_agent_idxs};
  }
};

}  // namespace libMultiRobotPlanning