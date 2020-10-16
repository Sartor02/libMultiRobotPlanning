#pragma once

#include <unordered_set>

#include <boost/functional/hash.hpp>

#include <libMultiRobotPlanning/neighbor.hpp>
#include <libMultiRobotPlanning/planresult.hpp>
#include <libMultiRobotPlanning/cbs.hpp>

namespace naive_cbs_wampf_impl {

using libMultiRobotPlanning::Neighbor;
using libMultiRobotPlanning::PlanResult;

struct CBSState {
  CBSState(int time, int x, int y) : time(time), x(x), y(y) {}

  bool operator==(const CBSState& s) const {
    return time == s.time && x == s.x && y == s.y;
  }

  bool equalExceptTime(const CBSState& s) const { return x == s.x && y == s.y; }

  friend std::ostream& operator<<(std::ostream& os, const CBSState& s) {
    return os << s.time << ": (" << s.x << "," << s.y << ")";
    // return os << "(" << s.x << "," << s.y << ")";
  }

  int time;
  int x;
  int y;
};
}

namespace std {
template <>
struct hash<naive_cbs_wampf_impl::CBSState> {
  size_t operator()(const naive_cbs_wampf_impl::CBSState& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.time);
    boost::hash_combine(seed, s.x);
    boost::hash_combine(seed, s.y);
    return seed;
  }
};
}  // namespace std

namespace naive_cbs_wampf_impl {
enum class CBSAction {
  Up,
  Down,
  Left,
  Right,
  Wait,
};


std::ostream& operator<<(std::ostream& os, const naive_cbs_wampf_impl::CBSAction& a) {
  switch (a) {
    case CBSAction::Up:
      os << "Up";
      break;
    case CBSAction::Down:
      os << "Down";
      break;
    case CBSAction::Left:
      os << "Left";
      break;
    case CBSAction::Right:
      os << "Right";
      break;
    case CBSAction::Wait:
      os << "Wait";
      break;
  }
  return os;
}
}


namespace naive_cbs_wampf_impl {
struct Conflict {
  enum Type {
    Vertex,
    Edge,
  };

  int time;
  size_t agent1;
  size_t agent2;
  Type type;

  int x1;
  int y1;
  int x2;
  int y2;

  friend std::ostream& operator<<(std::ostream& os, const Conflict& c) {
    switch (c.type) {
      case Vertex:
        return os << c.time << ": Vertex(" << c.x1 << "," << c.y1 << ")";
      case Edge:
        return os << c.time << ": Edge(" << c.x1 << "," << c.y1 << "," << c.x2
                  << "," << c.y2 << ")";
    }
    return os;
  }
};


struct VertexConstraint {
  VertexConstraint(int time, int x, int y) : time(time), x(x), y(y) {}
  int time;
  int x;
  int y;

  bool operator<(const VertexConstraint& other) const {
    return std::tie(time, x, y) < std::tie(other.time, other.x, other.y);
  }

  bool operator==(const VertexConstraint& other) const {
    return std::tie(time, x, y) == std::tie(other.time, other.x, other.y);
  }

  friend std::ostream& operator<<(std::ostream& os, const VertexConstraint& c) {
    return os << "VC(" << c.time << "," << c.x << "," << c.y << ")";
  }
};
}

namespace std {
template <>
struct hash<naive_cbs_wampf_impl::VertexConstraint> {
  size_t operator()(const naive_cbs_wampf_impl::VertexConstraint& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.time);
    boost::hash_combine(seed, s.x);
    boost::hash_combine(seed, s.y);
    return seed;
  }
};
}  // namespace std

namespace naive_cbs_wampf_impl {
struct EdgeConstraint {
  EdgeConstraint(int time, int x1, int y1, int x2, int y2)
      : time(time), x1(x1), y1(y1), x2(x2), y2(y2) {}
  int time;
  int x1;
  int y1;
  int x2;
  int y2;

  bool operator<(const EdgeConstraint& other) const {
    return std::tie(time, x1, y1, x2, y2) <
           std::tie(other.time, other.x1, other.y1, other.x2, other.y2);
  }

  bool operator==(const EdgeConstraint& other) const {
    return std::tie(time, x1, y1, x2, y2) ==
           std::tie(other.time, other.x1, other.y1, other.x2, other.y2);
  }

  friend std::ostream& operator<<(std::ostream& os, const EdgeConstraint& c) {
    return os << "EC(" << c.time << "," << c.x1 << "," << c.y1 << "," << c.x2
              << "," << c.y2 << ")";
  }
};

}

namespace std {
template <>
struct hash<naive_cbs_wampf_impl::EdgeConstraint> {
  size_t operator()(const naive_cbs_wampf_impl::EdgeConstraint& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.time);
    boost::hash_combine(seed, s.x1);
    boost::hash_combine(seed, s.y1);
    boost::hash_combine(seed, s.x2);
    boost::hash_combine(seed, s.y2);
    return seed;
  }
};
}  // namespace std

namespace naive_cbs_wampf_impl {
struct Constraints {
  std::unordered_set<VertexConstraint> vertexConstraints;
  std::unordered_set<EdgeConstraint> edgeConstraints;

  void add(const Constraints& other) {
    vertexConstraints.insert(other.vertexConstraints.begin(),
                             other.vertexConstraints.end());
    edgeConstraints.insert(other.edgeConstraints.begin(),
                           other.edgeConstraints.end());
  }

  bool overlap(const Constraints& other) {
    std::vector<VertexConstraint> vertexIntersection;
    std::vector<EdgeConstraint> edgeIntersection;
    std::set_intersection(vertexConstraints.begin(), vertexConstraints.end(),
                          other.vertexConstraints.begin(),
                          other.vertexConstraints.end(),
                          std::back_inserter(vertexIntersection));
    std::set_intersection(edgeConstraints.begin(), edgeConstraints.end(),
                          other.edgeConstraints.begin(),
                          other.edgeConstraints.end(),
                          std::back_inserter(edgeIntersection));
    return !vertexIntersection.empty() || !edgeIntersection.empty();
  }

  friend std::ostream& operator<<(std::ostream& os, const Constraints& c) {
    for (const auto& vc : c.vertexConstraints) {
      os << vc << std::endl;
    }
    for (const auto& ec : c.edgeConstraints) {
      os << ec << std::endl;
    }
    return os;
  }
};

}

namespace naive_cbs_wampf_impl {
template <typename Location>
class NaiveCBSEnvironment {
 public:
  NaiveCBSEnvironment(size_t dimx, size_t dimy, const std::unordered_set<Location>& obstacles,
              std::vector<Location> goals)
      : m_dimx(dimx),
        m_dimy(dimy),
        m_obstacles(obstacles),
        m_goals(std::move(goals)),
        m_agentIdx(0),
        m_constraints(nullptr),
        m_lastGoalConstraint(-1),
        m_highLevelExpanded(0),
        m_lowLevelExpanded(0) {
    // computeHeuristic();
  }

  NaiveCBSEnvironment(const NaiveCBSEnvironment&) = delete;
  NaiveCBSEnvironment& operator=(const NaiveCBSEnvironment&) = delete;

  void setLowLevelContext(size_t agentIdx, const Constraints* constraints) {
    assert(constraints);  // NOLINT
    m_agentIdx = agentIdx;
    m_constraints = constraints;
    m_lastGoalConstraint = -1;
    for (const auto& vc : constraints->vertexConstraints) {
      if (vc.x == m_goals[m_agentIdx].x && vc.y == m_goals[m_agentIdx].y) {
        m_lastGoalConstraint = std::max(m_lastGoalConstraint, vc.time);
      }
    }
  }

  int admissibleHeuristic(const CBSState& s) {
    // std::cout << "H: " <<  s << " " << m_heuristic[m_agentIdx][s.x + m_dimx *
    // s.y] << std::endl;
    // return m_heuristic[m_agentIdx][s.x + m_dimx * s.y];
    return std::abs(s.x - m_goals[m_agentIdx].x) +
           std::abs(s.y - m_goals[m_agentIdx].y);
  }

  bool isSolution(const CBSState& s) {
    return s.x == m_goals[m_agentIdx].x && s.y == m_goals[m_agentIdx].y &&
           s.time > m_lastGoalConstraint;
  }

  void getNeighbors(const CBSState& s,
                    std::vector<Neighbor<CBSState, CBSAction, int> >& neighbors) {
    neighbors.clear();
    {
      CBSState n(s.time + 1, s.x, s.y);
      if (CBSStateValid(n) && transitionValid(s, n)) {
        neighbors.emplace_back(
            Neighbor<CBSState, CBSAction, int>(n, CBSAction::Wait, 1));
      }
    }
    {
      CBSState n(s.time + 1, s.x - 1, s.y);
      if (CBSStateValid(n) && transitionValid(s, n)) {
        neighbors.emplace_back(
            Neighbor<CBSState, CBSAction, int>(n, CBSAction::Left, 1));
      }
    }
    {
      CBSState n(s.time + 1, s.x + 1, s.y);
      if (CBSStateValid(n) && transitionValid(s, n)) {
        neighbors.emplace_back(
            Neighbor<CBSState, CBSAction, int>(n, CBSAction::Right, 1));
      }
    }
    {
      CBSState n(s.time + 1, s.x, s.y + 1);
      if (CBSStateValid(n) && transitionValid(s, n)) {
        neighbors.emplace_back(Neighbor<CBSState, CBSAction, int>(n, CBSAction::Up, 1));
      }
    }
    {
      CBSState n(s.time + 1, s.x, s.y - 1);
      if (CBSStateValid(n) && transitionValid(s, n)) {
        neighbors.emplace_back(
            Neighbor<CBSState, CBSAction, int>(n, CBSAction::Down, 1));
      }
    }
  }

  bool getFirstConflict(
      const std::vector<PlanResult<CBSState, CBSAction, int> >& solution,
      Conflict& result) {
    int max_t = 0;
    for (const auto& sol : solution) {
      max_t = std::max<int>(max_t, sol.states.size() - 1);
    }

    for (int t = 0; t < max_t; ++t) {
      // check drive-drive vertex collisions
      for (size_t i = 0; i < solution.size(); ++i) {
        CBSState CBSState1 = getCBSState(i, solution, t);
        for (size_t j = i + 1; j < solution.size(); ++j) {
          CBSState CBSState2 = getCBSState(j, solution, t);
          if (CBSState1.equalExceptTime(CBSState2)) {
            result.time = t;
            result.agent1 = i;
            result.agent2 = j;
            result.type = Conflict::Vertex;
            result.x1 = CBSState1.x;
            result.y1 = CBSState1.y;
            // std::cout << "VC " << t << "," << CBSState1.x << "," << CBSState1.y <<
            // std::endl;
            return true;
          }
        }
      }
      // drive-drive edge (swap)
      for (size_t i = 0; i < solution.size(); ++i) {
        CBSState CBSState1a = getCBSState(i, solution, t);
        CBSState CBSState1b = getCBSState(i, solution, t + 1);
        for (size_t j = i + 1; j < solution.size(); ++j) {
          CBSState CBSState2a = getCBSState(j, solution, t);
          CBSState CBSState2b = getCBSState(j, solution, t + 1);
          if (CBSState1a.equalExceptTime(CBSState2b) &&
              CBSState1b.equalExceptTime(CBSState2a)) {
            result.time = t;
            result.agent1 = i;
            result.agent2 = j;
            result.type = Conflict::Edge;
            result.x1 = CBSState1a.x;
            result.y1 = CBSState1a.y;
            result.x2 = CBSState1b.x;
            result.y2 = CBSState1b.y;
            return true;
          }
        }
      }
    }

    return false;
  }

  void createConstraintsFromConflict(
      const Conflict& conflict, std::map<size_t, Constraints>& constraints) {
    if (conflict.type == Conflict::Vertex) {
      Constraints c1;
      c1.vertexConstraints.emplace(
          VertexConstraint(conflict.time, conflict.x1, conflict.y1));
      constraints[conflict.agent1] = c1;
      constraints[conflict.agent2] = c1;
    } else if (conflict.type == Conflict::Edge) {
      Constraints c1;
      c1.edgeConstraints.emplace(EdgeConstraint(
          conflict.time, conflict.x1, conflict.y1, conflict.x2, conflict.y2));
      constraints[conflict.agent1] = c1;
      Constraints c2;
      c2.edgeConstraints.emplace(EdgeConstraint(
          conflict.time, conflict.x2, conflict.y2, conflict.x1, conflict.y1));
      constraints[conflict.agent2] = c2;
    }
  }

  void onExpandHighLevelNode(int /*cost*/) { m_highLevelExpanded++; }

  void onExpandLowLevelNode(const CBSState& /*s*/, int /*fScore*/,
                            int /*gScore*/) {
    m_lowLevelExpanded++;
  }

  int highLevelExpanded() { return m_highLevelExpanded; }

  int lowLevelExpanded() const { return m_lowLevelExpanded; }

 private:
  CBSState getCBSState(size_t agentIdx,
                 const std::vector<PlanResult<CBSState, CBSAction, int> >& solution,
                 size_t t) {
    assert(agentIdx < solution.size());
    if (t < solution[agentIdx].states.size()) {
      return solution[agentIdx].states[t].first;
    }
    assert(!solution[agentIdx].states.empty());
    return solution[agentIdx].states.back().first;
  }

  bool CBSStateValid(const CBSState& s) {
    assert(m_constraints);
    const auto& con = m_constraints->vertexConstraints;
    return s.x >= 0 && s.x < m_dimx && s.y >= 0 && s.y < m_dimy &&
           m_obstacles.find({s.x, s.y}) == m_obstacles.end() &&
           con.find(VertexConstraint(s.time, s.x, s.y)) == con.end();
  }

  bool transitionValid(const CBSState& s1, const CBSState& s2) {
    assert(m_constraints);
    const auto& con = m_constraints->edgeConstraints;
    return con.find(EdgeConstraint(s1.time, s1.x, s1.y, s2.x, s2.y)) ==
           con.end();
  }

 private:
  int m_dimx;
  int m_dimy;
  const std::unordered_set<Location>& m_obstacles;
  std::vector<Location> m_goals;
  // std::vector< std::vector<int> > m_heuristic;
  size_t m_agentIdx;
  const Constraints* m_constraints;
  int m_lastGoalConstraint;
  int m_highLevelExpanded;
  int m_lowLevelExpanded;
};
}