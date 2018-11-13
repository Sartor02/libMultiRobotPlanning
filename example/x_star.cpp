#include <fstream>
#include <iostream>

#include <boost/functional/hash.hpp>
#include <boost/program_options.hpp>

#include <yaml-cpp/yaml.h>

#include <signal.h>
#include <libMultiRobotPlanning/helpers.hpp>
#include <libMultiRobotPlanning/x_star.hpp>
#include "timer.hpp"

using libMultiRobotPlanning::Neighbor;
using libMultiRobotPlanning::PlanResult;
using libMultiRobotPlanning::XStar;

void FatalSignalHandler(const int signo) {
  fprintf(stderr,
          "Received fatal signal %s, firing custom stack trace:\n",
          strsignal(signo));
  fflush(stderr);
  PrintStackTrace();
  exit(1);
}

template <typename T>
T L1Norm(T x1, T y1, T x2, T y2) {
  return std::abs(x1 - x2) + std::abs(y1 - y2);
}

template <typename T>
T linfNorm(T x1, T y1, T x2, T y2) {
  return std::max(std::abs(x1 - x2), std::abs(y1 - y2));
}

struct State {
  State() : time(0), x(0), y(0) {}
  State(int time, int x, int y) : time(time), x(x), y(y) {}
  State(const State& s) : time(s.time), x(s.x), y(s.y) {}

  bool operator==(const State& s) const {
    return time == s.time && x == s.x && y == s.y;
  }

  bool equalExceptTime(const State& s) const { return x == s.x && y == s.y; }

  friend std::ostream& operator<<(std::ostream& os, const State& s) {
    return os << s.time << ": (" << s.x << "," << s.y << ")";
    // return os << "(" << s.x << "," << s.y << ")";
  }

  int time;
  int x;
  int y;
};

namespace std {
template <>
struct hash<State> {
  size_t operator()(const State& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.time);
    boost::hash_combine(seed, s.x);
    boost::hash_combine(seed, s.y);
    return seed;
  }
};
}  // namespace std

///
enum class Action {
  Up,
  Down,
  Left,
  Right,
  Wait,
};

std::ostream& operator<<(std::ostream& os, const Action& a) {
  switch (a) {
    case Action::Up:
      os << "Up";
      break;
    case Action::Down:
      os << "Down";
      break;
    case Action::Left:
      os << "Left";
      break;
    case Action::Right:
      os << "Right";
      break;
    case Action::Wait:
      os << "Wait";
      break;
  }
  return os;
}

///

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

  bool operator==(const Conflict& other) const {
    switch (type) {
      case Vertex:
        return (time == other.time) && (agent1 == other.agent1) &&
               (agent2 == other.agent2) && (type == other.type) &&
               (x1 == other.x1) && (y1 == other.y1);
      case Edge:
        return (time == other.time) && (agent1 == other.agent1) &&
               (agent2 == other.agent2) && (type == other.type) &&
               (x1 == other.x1) && (x2 == other.x2) && (y1 == other.y1) &&
               (y2 == other.y2);
    }
  };

  bool operator!=(const Conflict& other) const { return !(*this == other); }

  friend std::ostream& operator<<(std::ostream& os, const Conflict& c) {
    switch (c.type) {
      case Vertex:
        return os << c.time << ": Vertex(" << c.x1 << "," << c.y1
                  << ") Between: " << c.agent1 << ", " << c.agent2;
      case Edge:
        return os << c.time << ": Edge(" << c.x1 << "," << c.y1 << "," << c.x2
                  << "," << c.y2 << ") Between: " << c.agent1 << ", "
                  << c.agent2;
    }
    return os;
  }
};

struct WindowBox {
  using GridVertex = std::pair<int, int>;
  GridVertex upper_left;
  GridVertex lower_right;
  int expansion_delta;

  WindowBox() : upper_left(0, 0), lower_right(0, 0), expansion_delta(0) {}
  WindowBox(const GridVertex& center, const int start_radius)
      : upper_left({center.first - start_radius, center.second + start_radius}),
        lower_right(
            {center.first + start_radius, center.second - start_radius}) {}
  WindowBox(const GridVertex& upper_right,
            const GridVertex& lower_left,
            const int& expansion_delta)
      : upper_left(upper_right),
        lower_right(lower_left),
        expansion_delta(expansion_delta) {}

  void Expand() {
    upper_left.first -= expansion_delta;
    upper_left.second += expansion_delta;
    lower_right.first += expansion_delta;
    lower_right.second -= expansion_delta;
  }

  bool Inside(const GridVertex& p) const { return InXRange(p) && InYRange(p); }

  bool Intersects(const WindowBox& other) const {
    if (Inside(other.lower_right)) {
      return true;
    }
    if (Inside(other.upper_left)) {
      return true;
    }
    const GridVertex other_upper_right = other.ComputeUpperRight();
    if (Inside(other_upper_right)) {
      return true;
    }
    const GridVertex other_lower_left = other.ComputeLowerLeft();
    if (Inside(other_lower_left)) {
      return true;
    }
    return false;
  }

  WindowBox Merge(const WindowBox& other) const {
    const GridVertex new_upper_left(
        std::min(upper_left.first, other.upper_left.first),
        std::max(upper_left.second, other.upper_left.second));
    const GridVertex new_lower_right(
        std::max(lower_right.first, other.lower_right.first),
        std::min(lower_right.second, other.lower_right.second));
    return {new_upper_left, new_lower_right, expansion_delta};
  }

  friend std::ostream& operator<<(std::ostream& os, const GridVertex& v) {
    os << "(" << v.first << ", " << v.second << ")";
    return os;
  }

 private:
  bool InXRange(const GridVertex& p) const {
    return (p.first <= lower_right.first && p.first >= upper_left.first);
  }
  bool InYRange(const GridVertex& p) const {
    return (p.second >= lower_right.second && p.second <= upper_left.second);
  }

  GridVertex ComputeUpperRight() const {
    return {lower_right.first, upper_left.second};
  }

  GridVertex ComputeLowerLeft() const {
    return {upper_left.first, lower_right.second};
  }
};

struct Window {
  static constexpr int kRadius = 1;
  static constexpr int kTimeDelta = 3;
  WindowBox box;
  int time;
  std::vector<size_t> agents;
  size_t start_index;
  std::vector<size_t> goal_indices;

  Window() = delete;
  Window(const Conflict& c)
      : box({c.x1, c.y1}, kRadius),
        time(c.time),
        agents({c.agent1, c.agent2}),
        start_index(0) {}
  Window(const WindowBox& box,
         const int& time,
         const std::vector<size_t>& agents)
      : box(box), time(time), agents(agents), start_index(0) {}

  void Expand() { box.Expand(); }

  bool IsInWindow(const int qx, const int qy) const {
    return box.Inside({qx, qy});
  }

  void DeDupAgents() {
    std::sort(agents.begin(), agents.end());
    agents.erase(std::unique(agents.begin(), agents.end()), agents.end());
  }

  bool Intersects(const Window& other) const {
    return box.Intersects(other.box);
  }

  bool AgentsOverlap(const Window& other) const {
    std::vector<size_t> overlap;
    std::set_intersection(agents.begin(),
                          agents.end(),
                          other.agents.begin(),
                          other.agents.end(),
                          std::back_inserter(overlap));
    return !overlap.empty();
  }

  Window Merge(const Window& other) const {
    const WindowBox new_box = box.Merge(other.box);
    std::vector<size_t> new_agents(agents);
    std::copy(other.agents.begin(),
              other.agents.end(),
              std::back_inserter(new_agents));
    Window new_window(new_box, (time + other.time) / 2, new_agents);
    new_window.DeDupAgents();
    return new_window;
  }

  bool IncludeConflictIfApplicable(const Conflict& c) {
    switch (c.type) {
      case Conflict::Vertex:
        if (IsInWindow(c.x1, c.y1) && std::abs(c.time - time) <= kTimeDelta) {
          if (std::find(agents.begin(), agents.end(), c.agent1) ==
                  agents.end() &&
              std::find(agents.begin(), agents.end(), c.agent2) ==
                  agents.end()) {
            return false;
          }
          agents.push_back(c.agent1);
          agents.push_back(c.agent2);
          DeDupAgents();
          std::cout << "Included " << c << " into " << *this << '\n';
          return true;
        }
        break;
      case Conflict::Edge:
        if (IsInWindow(c.x1, c.y1) && std::abs(c.time - time) <= kTimeDelta) {
          if (std::find(agents.begin(), agents.end(), c.agent1) ==
                  agents.end() &&
              std::find(agents.begin(), agents.end(), c.agent2) ==
                  agents.end()) {
            return false;
          }

          agents.push_back(c.agent1);
          agents.push_back(c.agent2);
          DeDupAgents();
          std::cout << "Included " << c << " into " << *this << '\n';
          return true;
        }
        if (IsInWindow(c.x2, c.y2) && std::abs(c.time - time) <= kTimeDelta) {
          if (std::find(agents.begin(), agents.end(), c.agent1) ==
                  agents.end() &&
              std::find(agents.begin(), agents.end(), c.agent2) ==
                  agents.end()) {
            return false;
          }

          agents.push_back(c.agent1);
          agents.push_back(c.agent2);
          DeDupAgents();
          std::cout << "Included " << c << " into " << *this << '\n';
          return true;
        }
        break;
    }
    return false;
  }

  friend std::ostream& operator<<(std::ostream& os, const Window& w) {
    assert(w.agents.size() == w.goal_indices.size());
    os << "LR: " << w.box.lower_right.first << ", " << w.box.lower_right.second
       << " UL: " << w.box.upper_left.first << ", " << w.box.upper_left.second
       << "; Time: " << w.time << "; Agents: ";
    for (const auto& e : w.agents) {
      os << e << " ";
    }
    os << "Start index: " << w.start_index << " Goal indices: ";
    for (const auto& i : w.goal_indices) {
      os << i << " ";
    }
    return os;
  }
};

struct Location {
  Location(int x, int y) : x(x), y(y) {}
  int x;
  int y;

  bool operator<(const Location& other) const {
    return std::tie(x, y) < std::tie(other.x, other.y);
  }

  bool operator==(const Location& other) const {
    return std::tie(x, y) == std::tie(other.x, other.y);
  }

  friend std::ostream& operator<<(std::ostream& os, const Location& c) {
    return os << "(" << c.x << "," << c.y << ")";
  }
};

namespace std {
template <>
struct hash<Location> {
  size_t operator()(const Location& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.x);
    boost::hash_combine(seed, s.y);
    return seed;
  }
};
}  // namespace std

///
class Environment {
 public:
  Environment(size_t dimx,
              size_t dimy,
              std::unordered_set<Location> obstacles,
              std::vector<Location> goals)
      : m_dimx(dimx),
        m_dimy(dimy),
        m_obstacles(std::move(obstacles)),
        m_goals(std::move(goals)),
        m_lastGoalConstraint(-1),
        m_highLevelExpanded(0),
        m_lowLevelExpanded(0) {
    // computeHeuristic();
  }

  Environment(const Environment&) = delete;
  Environment& operator=(const Environment&) = delete;

  int admissibleHeuristic(const State& s, const size_t agent_index) {
    // std::cout << "H: " <<  s << " " << m_heuristic[m_agentIdx][s.x + m_dimx *
    // s.y] << std::endl;
    // return m_heuristic[m_agentIdx][s.x + m_dimx * s.y];
    return std::abs(s.x - m_goals[agent_index].x) +
           std::abs(s.y - m_goals[agent_index].y);
  }

  int AdmissibleHeuristic(const State& s1, const State& s2) {
    // std::cout << "H: " <<  s << " " << m_heuristic[m_agentIdx][s.x + m_dimx *
    // s.y] << std::endl;
    // return m_heuristic[m_agentIdx][s.x + m_dimx * s.y];
    return L1Norm(s1.x, s1.y, s2.x, s2.y);
  }

  bool isSolution(const State& s, const size_t agent_index) {
    return s.x == m_goals[agent_index].x && s.y == m_goals[agent_index].y &&
           s.time > m_lastGoalConstraint;
  }

  void getNeighbors(const State& s,
                    const size_t agent_index,
                    std::vector<Neighbor<State, Action, int>>& neighbors) {
    // std::cout << "#VC " << constraints.vertexConstraints.size() << std::endl;
    // for(const auto& vc : constraints.vertexConstraints) {
    //   std::cout << "  " << vc.time << "," << vc.x << "," << vc.y <<
    //   std::endl;
    // }
    neighbors.clear();

    if (isSolution(s, agent_index)) {
      State n(s.time + 1, s.x, s.y);
      if (stateValid(n) && transitionValid(s, n)) {
        neighbors.emplace_back(
            Neighbor<State, Action, int>(n, Action::Wait, 0));
      }
    }

    {
      State n(s.time + 1, s.x, s.y);
      if (stateValid(n) && transitionValid(s, n)) {
        neighbors.emplace_back(
            Neighbor<State, Action, int>(n, Action::Wait, 1));
      }
    }
    {
      State n(s.time + 1, s.x - 1, s.y);
      if (stateValid(n) && transitionValid(s, n)) {
        neighbors.emplace_back(
            Neighbor<State, Action, int>(n, Action::Left, 1));
      }
    }
    {
      State n(s.time + 1, s.x + 1, s.y);
      if (stateValid(n) && transitionValid(s, n)) {
        neighbors.emplace_back(
            Neighbor<State, Action, int>(n, Action::Right, 1));
      }
    }
    {
      State n(s.time + 1, s.x, s.y + 1);
      if (stateValid(n) && transitionValid(s, n)) {
        neighbors.emplace_back(Neighbor<State, Action, int>(n, Action::Up, 1));
      }
    }
    {
      State n(s.time + 1, s.x, s.y - 1);
      if (stateValid(n) && transitionValid(s, n)) {
        neighbors.emplace_back(
            Neighbor<State, Action, int>(n, Action::Down, 1));
      }
    }
  }

  bool getFirstConflict(
      const std::vector<PlanResult<State, Action, int>>& solution,
      Conflict& result) {
    int max_t = 0;
    for (const auto& sol : solution) {
      max_t = std::max<int>(max_t, sol.states.size() - 1);
    }

    for (int t = 0; t < max_t; ++t) {
      // check drive-drive vertex collisions
      for (size_t i = 0; i < solution.size(); ++i) {
        State state1 = getState(i, solution, t);
        for (size_t j = i + 1; j < solution.size(); ++j) {
          State state2 = getState(j, solution, t);
          if (state1.equalExceptTime(state2)) {
            result.time = t;
            result.agent1 = i;
            result.agent2 = j;
            result.type = Conflict::Vertex;
            result.x1 = state1.x;
            result.y1 = state1.y;
            // std::cout << "VC " << t << "," << state1.x << "," << state1.y <<
            // std::endl;
            return true;
          }
        }
      }
      // drive-drive edge (swap)
      for (size_t i = 0; i < solution.size(); ++i) {
        State state1a = getState(i, solution, t);
        State state1b = getState(i, solution, t + 1);
        for (size_t j = i + 1; j < solution.size(); ++j) {
          State state2a = getState(j, solution, t);
          State state2b = getState(j, solution, t + 1);
          if (state1a.equalExceptTime(state2b) &&
              state1b.equalExceptTime(state2a)) {
            result.time = t;
            result.agent1 = i;
            result.agent2 = j;
            result.type = Conflict::Edge;
            result.x1 = state1a.x;
            result.y1 = state1a.y;
            result.x2 = state1b.x;
            result.y2 = state1b.y;
            return true;
          }
        }
      }
    }

    return false;
  }

  std::vector<Conflict> getAllConflicts(
      const std::vector<PlanResult<State, Action, int>>& solution) {
    std::vector<Conflict> conflicts;
    int max_t = 0;
    for (const auto& sol : solution) {
      max_t = std::max<int>(max_t, sol.states.size() - 1);
    }

    for (int t = 0; t < max_t; ++t) {
      // check drive-drive vertex collisions
      for (size_t i = 0; i < solution.size(); ++i) {
        State state1 = getState(i, solution, t);
        for (size_t j = i + 1; j < solution.size(); ++j) {
          State state2 = getState(j, solution, t);
          if (state1.equalExceptTime(state2)) {
            Conflict result;
            result.time = t;
            result.agent1 = i;
            result.agent2 = j;
            result.type = Conflict::Vertex;
            result.x1 = state1.x;
            result.y1 = state1.y;
            conflicts.push_back(result);
          }
        }
      }
      // drive-drive edge (swap)
      for (size_t i = 0; i < solution.size(); ++i) {
        State state1a = getState(i, solution, t);
        State state1b = getState(i, solution, t + 1);
        for (size_t j = i + 1; j < solution.size(); ++j) {
          State state2a = getState(j, solution, t);
          State state2b = getState(j, solution, t + 1);
          if (state1a.equalExceptTime(state2b) &&
              state1b.equalExceptTime(state2a)) {
            Conflict result;
            result.time = t;
            result.agent1 = i;
            result.agent2 = j;
            result.type = Conflict::Edge;
            result.x1 = state1a.x;
            result.y1 = state1a.y;
            result.x2 = state1b.x;
            result.y2 = state1b.y;
            conflicts.push_back(result);
          }
        }
      }
    }

    return conflicts;
  }

  size_t AddConflictToWindows(Conflict& c, std::vector<Window>* windows) {
    for (size_t i = 0; i < windows->size(); ++i) {
      Window& w = (*windows)[i];
      if (w.IncludeConflictIfApplicable(c)) {
        std::cout << "Merged into existing window\n";
        return i;
      }
    }

    std::cout << "Added new window\n";
    windows->push_back({c});
    return windows->size() - 1;
  }

  void SetWindowIndices(
      Window* w,
      const std::vector<PlanResult<State, Action, int>>& individual_solutions) {
    w->start_index = std::numeric_limits<size_t>::max();
    w->goal_indices.clear();
    for (const size_t& agent_index : w->agents) {
      const auto& solution = individual_solutions[agent_index];
      for (size_t i = 0; i < solution.states.size(); ++i) {
        const std::pair<State, int>& step = solution.states[i];
        if (w->IsInWindow(step.first.x, step.first.y)) {
          w->start_index = std::min(w->start_index, i);
          break;
        }
      }

      for (int i = solution.states.size() - 1; i >= 0; --i) {
        const std::pair<State, int>& step = solution.states[i];
        if (w->IsInWindow(step.first.x, step.first.y)) {
          w->goal_indices.push_back(i);
          break;
        }
      }
    }
  }

  void onExpandHighLevelNode(int /*cost*/) { m_highLevelExpanded++; }

  void onExpandLowLevelNode(const State& /*s*/,
                            int /*fScore*/,
                            int /*gScore*/) {
    m_lowLevelExpanded++;
  }

  int highLevelExpanded() { return m_highLevelExpanded; }

  int lowLevelExpanded() const { return m_lowLevelExpanded; }

 private:
  State getState(size_t agentIdx,
                 const std::vector<PlanResult<State, Action, int>>& solution,
                 size_t t) {
    assert(agentIdx < solution.size());
    if (t < solution[agentIdx].states.size()) {
      return solution[agentIdx].states[t].first;
    }
    assert(!solution[agentIdx].states.empty());
    return solution[agentIdx].states.back().first;
  }

  bool stateValid(const State& s) {
    return s.x >= 0 && s.x < m_dimx && s.y >= 0 && s.y < m_dimy &&
           m_obstacles.find(Location(s.x, s.y)) == m_obstacles.end();
    ;
  }

  bool transitionValid(const State&, const State&) { return true; }
#if 0
  // We use another A* search for simplicity
  // we compute the shortest path to each goal by using the fact that our getNeighbor function is
  // symmetric and by not terminating the AStar search until the queue is empty
  void computeHeuristic()
  {
    class HeuristicEnvironment
    {
    public:
      HeuristicEnvironment(
        size_t dimx,
        size_t dimy,
        const std::unordered_set<Location>& obstacles,
        std::vector<int>* heuristic)
        : m_dimx(dimx)
        , m_dimy(dimy)
        , m_obstacles(obstacles)
        , m_heuristic(heuristic)
      {
      }

      int admissibleHeuristic(
        const Location& s)
      {
        return 0;
      }

      bool isSolution(
        const Location& s)
      {
        return false;
      }

      void getNeighbors(
        const Location& s,
        std::vector<Neighbor<Location, Action, int> >& neighbors)
      {
        neighbors.clear();

        {
          Location n(s.x-1, s.y);
          if (stateValid(n)) {
            neighbors.emplace_back(Neighbor<Location, Action, int>(n, Action::Left, 1));
          }
        }
        {
          Location n(s.x+1, s.y);
          if (stateValid(n)) {
            neighbors.emplace_back(Neighbor<Location, Action, int>(n, Action::Right, 1));
          }
        }
        {
          Location n(s.x, s.y+1);
          if (stateValid(n)) {
            neighbors.emplace_back(Neighbor<Location, Action, int>(n, Action::Up, 1));
          }
        }
        {
          Location n(s.x, s.y-1);
          if (stateValid(n)) {
            neighbors.emplace_back(Neighbor<Location, Action, int>(n, Action::Down, 1));
          }
        }
      }

      void onExpandNode(
        const Location& s,
        int fScore,
        int gScore)
      {
      }

      void onDiscover(
        const Location& s,
        int fScore,
        int gScore)
      {
        (*m_heuristic)[s.x + m_dimx * s.y] = gScore;
      }

    private:
      bool stateValid(
        const Location& s)
      {
        return    s.x >= 0
               && s.x < m_dimx
               && s.y >= 0
               && s.y < m_dimy
               && m_obstacles.find(Location(s.x, s.y)) == m_obstacles.end();
      }

    private:
      int m_dimx;
      int m_dimy;
      const std::unordered_set<Location>& m_obstacles;
      std::vector<int>* m_heuristic;

    };

    m_heuristic.resize(m_goals.size());

    std::vector< Neighbor<State, Action, int> > neighbors;

    for (size_t i = 0; i < m_goals.size(); ++i) {
      m_heuristic[i].assign(m_dimx * m_dimy, std::numeric_limits<int>::max());
      HeuristicEnvironment henv(m_dimx, m_dimy, m_obstacles, &m_heuristic[i]);
      AStar<Location, Action, int, HeuristicEnvironment> astar(henv);
      PlanResult<Location, Action, int> dummy;
      astar.search(m_goals[i], dummy);
      m_heuristic[i][m_goals[i].x + m_dimx * m_goals[i].y] = 0;
    }
  }
#endif
 private:
  int m_dimx;
  int m_dimy;
  std::unordered_set<Location> m_obstacles;
  std::vector<Location> m_goals;
  // std::vector< std::vector<int> > m_heuristic;
  int m_lastGoalConstraint;
  int m_highLevelExpanded;
  int m_lowLevelExpanded;
};

int main(int argc, char* argv[]) {
  if (signal(SIGINT, FatalSignalHandler) == SIG_ERR) {
    std::cerr << "Cannot trap SIGINT\n";
    exit(-1);
  }

  if (signal(SIGSEGV, FatalSignalHandler) == SIG_ERR) {
    std::cerr << "Cannot trap SIGSEGV\n";
    exit(-1);
  }

  if (signal(SIGABRT, FatalSignalHandler) == SIG_ERR) {
    std::cerr << "Cannot trap SIGABRT\n";
    exit(-1);
  }

  namespace po = boost::program_options;
  // Declare the supported options.
  po::options_description desc("Allowed options");
  std::string inputFile;
  std::string outputFile;
  desc.add_options()("help", "produce help message")(
      "input,i",
      po::value<std::string>(&inputFile)->required(),
      "input file (YAML)")("output,o",
                           po::value<std::string>(&outputFile)->required(),
                           "output file (YAML)");

  try {
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help") != 0u) {
      std::cout << desc << "\n";
      return 0;
    }
  } catch (po::error& e) {
    std::cerr << e.what() << std::endl << std::endl;
    std::cerr << desc << std::endl;
    return 1;
  }

  YAML::Node config = YAML::LoadFile(inputFile);

  std::unordered_set<Location> obstacles;
  std::vector<Location> goals;
  std::vector<State> startStates;

  const auto& dim = config["map"]["dimensions"];
  int dimx = dim[0].as<int>();
  int dimy = dim[1].as<int>();

  for (const auto& node : config["map"]["obstacles"]) {
    obstacles.insert(Location(node[0].as<int>(), node[1].as<int>()));
  }

  for (const auto& node : config["agents"]) {
    const auto& start = node["start"];
    const auto& goal = node["goal"];
    startStates.emplace_back(State(0, start[0].as<int>(), start[1].as<int>()));
    // std::cout << "s: " << startStates.back() << std::endl;
    goals.emplace_back(Location(goal[0].as<int>(), goal[1].as<int>()));
  }

  Environment mapf(dimx, dimy, obstacles, goals);
  XStar<State, Action, int, Conflict, Environment, Window> x_star(mapf);
  std::vector<PlanResult<State, Action, int>> solution;

  Timer timer;
  bool success = x_star.search(startStates, solution);
  timer.stop();

  if (success) {
    std::cout << "Planning successful! " << std::endl;
    int cost = 0;
    int makespan = 0;
    for (const auto& s : solution) {
      cost += s.cost;
      makespan = std::max<int>(makespan, s.cost);
    }

    std::ofstream out(outputFile);
    out << "statistics:" << std::endl;
    out << "  cost: " << cost << std::endl;
    out << "  makespan: " << makespan << std::endl;
    out << "  runtime: " << timer.elapsedSeconds() << std::endl;
    out << "  highLevelExpanded: " << mapf.highLevelExpanded() << std::endl;
    out << "  lowLevelExpanded: " << mapf.lowLevelExpanded() << std::endl;
    out << "schedule:" << std::endl;
    for (size_t a = 0; a < solution.size(); ++a) {
      // std::cout << "Solution for: " << a << std::endl;
      // for (size_t i = 0; i < solution[a].actions.size(); ++i) {
      //   std::cout << solution[a].states[i].second << ": " <<
      //   solution[a].states[i].first << "->" << solution[a].actions[i].first
      //   << "(cost: " << solution[a].actions[i].second << ")" << std::endl;
      // }
      // std::cout << solution[a].states.back().second << ": " <<
      // solution[a].states.back().first << std::endl;

      out << "  agent" << a << ":" << std::endl;
      for (const auto& state : solution[a].states) {
        out << "    - x: " << state.first.x << std::endl
            << "      y: " << state.first.y << std::endl
            << "      t: " << state.first.time << std::endl;
      }
    }
  } else {
    std::cout << "Planning NOT successful!" << std::endl;
  }

  return 0;
}
