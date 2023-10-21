#include <fstream>
#include <iostream>

#include <boost/functional/hash.hpp>
#include <boost/program_options.hpp>

#include <yaml-cpp/yaml.h>

#include <libMultiRobotPlanning/cbs.hpp>
#include "timer.hpp"

using namespace std;
using libMultiRobotPlanning::CBS;
using libMultiRobotPlanning::Neighbor;
using libMultiRobotPlanning::PlanResult;

struct State
{
    State(int time, int x, int y) : time(time), x(x), y(y) {}

    bool operator==(const State& s) const
    {
        return time == s.time && x == s.x && y == s.y;
    }

    bool equalExceptTime(const State& s) const { return x == s.x && y == s.y; }

    friend ostream& operator<<(ostream& os, const State& s)
    {
        return os << s.time << ": (" << s.x << "," << s.y << ")";
      // return os << "(" << s.x << "," << s.y << ")";

    }

    int time;
    int x;
    int y;
};

namespace std
{
    template <>
    struct hash<State>
    {
        size_t operator()(const State& s) const
        {
            size_t seed = 0;
            boost::hash_combine(seed, s.time);
            boost::hash_combine(seed, s.x);
            boost::hash_combine(seed, s.y);

            return seed;
        }
    };
}

// 枚举类
enum class Action
{
    Up,
    Down,
    Left,
    Right,
    Wait,
};

ostream& operator<<(ostream& os, const Action& this_action)
{
    switch (this_action)
    {
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

struct Conflict
{
    enum Type
    {
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

    friend ostream& operator<<(ostream& os, const Conflict& c)
    {
        switch (c.type)
        {
          case Vertex:
            return os << c.time << ": Vertex(" << c.x1 << "," << c.y1 << ")";
          case Edge:
            return os << c.time << ": Edge(" << c.x1 << "," << c.y1 << "," << c.x2
                      << "," << c.y2 << ")";
        }

        return os;
    }
};

struct VertexConstraint
{
    VertexConstraint(int time, int x, int y) : time(time), x(x), y(y) {}
    int time;
    int x;
    int y;

    bool operator<(const VertexConstraint& other) const
    {
        return tie(time, x, y) < tie(other.time, other.x, other.y);
    }

    bool operator==(const VertexConstraint& other) const
    {
        return tie(time, x, y) == tie(other.time, other.x, other.y);
    }

    friend ostream& operator<<(ostream& os, const VertexConstraint& c)
    {
        return os << "VC(" << c.time << "," << c.x << "," << c.y << ")";
    }
};

namespace std
{
    template <>
    struct hash<VertexConstraint>
    {
        size_t operator()(const VertexConstraint& s) const
        {
            size_t seed = 0;
            boost::hash_combine(seed, s.time);
            boost::hash_combine(seed, s.x);
            boost::hash_combine(seed, s.y);
            return seed;
        }
    };
}

struct EdgeConstraint
{
    EdgeConstraint(int time, int x1, int y1, int x2, int y2)
      : time(time), x1(x1), y1(y1), x2(x2), y2(y2) {}
    int time;
    int x1;
    int y1;
    int x2;
    int y2;

    bool operator<(const EdgeConstraint& other) const
    {
        return tie(time, x1, y1, x2, y2) <
               tie(other.time, other.x1, other.y1, other.x2, other.y2);
    }

    bool operator==(const EdgeConstraint& other) const
    {
        return tie(time, x1, y1, x2, y2) ==
               tie(other.time, other.x1, other.y1, other.x2, other.y2);
    }

    friend ostream& operator<<(ostream& os, const EdgeConstraint& c)
    {
        return os << "EC(" << c.time << "," << c.x1 << "," << c.y1 << "," << c.x2
                  << "," << c.y2 << ")";
    }
};

namespace std
{
    template <>
    struct hash<EdgeConstraint>
    {
        size_t operator()(const EdgeConstraint& s) const
        {
            size_t seed = 0;
            boost::hash_combine(seed, s.time);
            boost::hash_combine(seed, s.x1);
            boost::hash_combine(seed, s.y1);
            boost::hash_combine(seed, s.x2);
            boost::hash_combine(seed, s.y2);

            return seed;
        }
    };
}

struct Constraints
{
  unordered_set<VertexConstraint> vertexConstraints;
  unordered_set<EdgeConstraint> edgeConstraints;

  void add(const Constraints& other) {
    vertexConstraints.insert(other.vertexConstraints.begin(),
                             other.vertexConstraints.end());
    edgeConstraints.insert(other.edgeConstraints.begin(),
                           other.edgeConstraints.end());
  }

  bool overlap(const Constraints& other) const {
    for (const auto& vc : vertexConstraints) {
      if (other.vertexConstraints.count(vc) > 0) {
        return true;
      }
    }
    for (const auto& ec : edgeConstraints) {
      if (other.edgeConstraints.count(ec) > 0) {
        return true;
      }
    }
    return false;
  }

  friend ostream& operator<<(ostream& os, const Constraints& c)
  {
    for (const auto& vc : c.vertexConstraints)
    {
      os << vc << endl;
    }

    for (const auto& ec : c.edgeConstraints)
    {
      os << ec << endl;
    }

    return os;
  }
};

struct Location
{
  Location(int x, int y) : x(x), y(y) {}
  int x;
  int y;

  bool operator<(const Location& other) const
  {
    return tie(x, y) < tie(other.x, other.y);
  }

  bool operator==(const Location& other) const
  {
    return tie(x, y) == tie(other.x, other.y);
  }

  friend ostream& operator<<(ostream& os, const Location& c)
  {
    return os << "(" << c.x << "," << c.y << ")";
  }
};

namespace std
{
    template <>
    struct hash<Location>
    {
        size_t operator()(const Location& s) const
        {
            size_t seed = 0;
            boost::hash_combine(seed, s.x);
            boost::hash_combine(seed, s.y);

            return seed;
        }
    };
}

///
class Environment
{
 public:
  Environment(size_t dimx, size_t dimy, unordered_set<Location> obstacles,
              vector<Location> goals, bool disappearAtGoal = false)
      : m_dimx(dimx),
        m_dimy(dimy),
        m_obstacles(move(obstacles)),
        m_goals(move(goals)),
        m_agentIdx(0),
        m_constraints(nullptr),
        m_lastGoalConstraint(-1),
        m_highLevelExpanded(0),
        m_lowLevelExpanded(0),
        m_disappearAtGoal(disappearAtGoal)
  {
  }

  Environment(const Environment&) = delete;
  Environment& operator=(const Environment&) = delete;

  void setLowLevelContext(size_t agentIdx, const Constraints* constraints) {
    assert(constraints);  // NOLINT
    m_agentIdx = agentIdx;
    m_constraints = constraints;
    m_lastGoalConstraint = -1;
    for (const auto& vc : constraints->vertexConstraints) {
      if (vc.x == m_goals[m_agentIdx].x && vc.y == m_goals[m_agentIdx].y) {
        m_lastGoalConstraint = max(m_lastGoalConstraint, vc.time);
      }
    }
  }

  int admissibleHeuristic(const State& s)
  {
    // cout << "H: " <<  s << " " << m_heuristic[m_agentIdx][s.x + m_dimx *
    // s.y] << endl;
    // return m_heuristic[m_agentIdx][s.x + m_dimx * s.y];
    return abs(s.x - m_goals[m_agentIdx].x) +
           abs(s.y - m_goals[m_agentIdx].y);
  }

  bool isSolution(const State& s)
  {
    return s.x == m_goals[m_agentIdx].x && s.y == m_goals[m_agentIdx].y &&
           s.time > m_lastGoalConstraint;
  }

  void getNeighbors(const State& s,
                    vector<Neighbor<State, Action, int> >& neighbors) {
    // cout << "#VC " << constraints.vertexConstraints.size() << endl;
    // for(const auto& vc : constraints.vertexConstraints) {
    //   cout << "  " << vc.time << "," << vc.x << "," << vc.y <<
    //   endl;
    // }
    neighbors.clear();
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
      const vector<PlanResult<State, Action, int> >& solution,
      Conflict& result)
  {
    int max_t = 0;
    for (const auto& sol : solution)
    {
      max_t = max<int>(max_t, sol.states.size() - 1);
    }

    for (int t = 0; t <= max_t; ++t)
    {
      // check drive-drive vertex collisions
      for (size_t i = 0; i < solution.size(); ++i)
      {
        State state1 = getState(i, solution, t);
        for (size_t j = i + 1; j < solution.size(); ++j)
        {
          State state2 = getState(j, solution, t);
          if (state1.equalExceptTime(state2))
          {
            result.time = t;
            result.agent1 = i;
            result.agent2 = j;
            result.type = Conflict::Vertex;
            result.x1 = state1.x;
            result.y1 = state1.y;
            // cout << "VC " << t << "," << state1.x << "," << state1.y <<
            // endl;
            return true;
          }
        }
      }
      // drive-drive edge (swap)
      for (size_t i = 0; i < solution.size(); ++i)
      {
        State state1a = getState(i, solution, t);
        State state1b = getState(i, solution, t + 1);
        for (size_t j = i + 1; j < solution.size(); ++j)
        {
          State state2a = getState(j, solution, t);
          State state2b = getState(j, solution, t + 1);
          if (state1a.equalExceptTime(state2b) &&
              state1b.equalExceptTime(state2a))
          {
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

  void createConstraintsFromConflict(
      const Conflict& conflict, map<size_t, Constraints>& constraints)
  {
    if (conflict.type == Conflict::Vertex)
    {
      Constraints c1;
      c1.vertexConstraints.emplace(
          VertexConstraint(conflict.time, conflict.x1, conflict.y1));
      constraints[conflict.agent1] = c1;
      constraints[conflict.agent2] = c1;
    }
    else if (conflict.type == Conflict::Edge)
    {
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

  void onExpandLowLevelNode(const State& /*s*/, int /*fScore*/,
                            int /*gScore*/) {
    m_lowLevelExpanded++;
  }

  int highLevelExpanded() { return m_highLevelExpanded; }

  int lowLevelExpanded() const { return m_lowLevelExpanded; }

 private:
  State getState(size_t agentIdx,
                 const vector<PlanResult<State, Action, int> >& solution,
                 size_t t)
  {
    assert(agentIdx < solution.size());
    if (t < solution[agentIdx].states.size())
    {
      return solution[agentIdx].states[t].first;
    }
    assert(!solution[agentIdx].states.empty());
    if (m_disappearAtGoal)
    {
      // This is a trick to avoid changing the rest of the code significantly
      // After an agent disappeared, put it at a unique but invalid position
      // This will cause all calls to equalExceptTime(.) to return false.
      return State(-1, -1 * (agentIdx + 1), -1);
    }

    return solution[agentIdx].states.back().first;
  }

  bool stateValid(const State& s)
  {
    assert(m_constraints);
    const auto& con = m_constraints->vertexConstraints;
    return s.x >= 0 && s.x < m_dimx && s.y >= 0 && s.y < m_dimy &&
           m_obstacles.find(Location(s.x, s.y)) == m_obstacles.end() &&
           con.find(VertexConstraint(s.time, s.x, s.y)) == con.end();
  }

  bool transitionValid(const State& s1, const State& s2)
  {
    assert(m_constraints);
    const auto& con = m_constraints->edgeConstraints;
    return con.find(EdgeConstraint(s1.time, s1.x, s1.y, s2.x, s2.y)) ==
           con.end();
  }
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
        const unordered_set<Location>& obstacles,
        vector<int>* heuristic)
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
        vector<Neighbor<Location, Action, int> >& neighbors)
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
      const unordered_set<Location>& m_obstacles;
      vector<int>* m_heuristic;

    };

    m_heuristic.resize(m_goals.size());

    vector< Neighbor<State, Action, int> > neighbors;

    for (size_t i = 0; i < m_goals.size(); ++i)
    {
      m_heuristic[i].assign(m_dimx * m_dimy, numeric_limits<int>::max());
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
  unordered_set<Location> m_obstacles;
  vector<Location> m_goals;
  // vector< vector<int> > m_heuristic;
  size_t m_agentIdx;
  const Constraints* m_constraints;
  int m_lastGoalConstraint;
  int m_highLevelExpanded;
  int m_lowLevelExpanded;
  bool m_disappearAtGoal;
};

int main(int argc, char* argv[])
{
  namespace po = boost::program_options;
  // Declare the supported options.
  po::options_description desc("Allowed options");
  string inputFile;
  string outputFile;
  bool disappearAtGoal;
  desc.add_options()("help", "produce help message")(
      "input,i", po::value<string>(&inputFile)->required(),
      "input file (YAML)")("output,o",
                           po::value<string>(&outputFile)->required(),
                           "output file (YAML)")(
      "disappear-at-goal", po::bool_switch(&disappearAtGoal), "make agents to disappear at goal rather than staying there");

  try {
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help") != 0u)
    {
      cout << desc << "\n";
      return 0;
    }
  }
  catch (po::error& e)
  {
    cerr << e.what() << endl << endl;
    cerr << desc << endl;
    return 1;
  }

  YAML::Node config = YAML::LoadFile(inputFile);

  unordered_set<Location> obstacles;
  vector<Location> goals;
  vector<State> startStates;

  const auto& dim = config["map"]["dimensions"];
  int dimx = dim[0].as<int>();
  int dimy = dim[1].as<int>();

  for (const auto& node : config["map"]["obstacles"]) {
    obstacles.insert(Location(node[0].as<int>(), node[1].as<int>()));
  }

  for (const auto& node : config["agents"])
  {
    const auto& start = node["start"];
    const auto& goal = node["goal"];
    startStates.emplace_back(State(0, start[0].as<int>(), start[1].as<int>()));
    // cout << "s: " << startStates.back() << endl;
    goals.emplace_back(Location(goal[0].as<int>(), goal[1].as<int>()));
  }

  // sanity check: no identical start states
  unordered_set<State> startStatesSet;
  for (const auto& s : startStates)
  {
    if (startStatesSet.find(s) != startStatesSet.end())
    {
      cout << "Identical start states detected -> no solution!" << endl;

      return 0;
    }

    startStatesSet.insert(s);
  }

  Environment mapf(dimx, dimy, obstacles, goals, disappearAtGoal);
  CBS<State, Action, int, Conflict, Constraints, Environment> cbs(mapf);
  vector<PlanResult<State, Action, int> > solution;

  Timer timer;
  bool success = cbs.search(startStates, solution);
  timer.stop();

  if (success)
  {
    cout << "Planning successful! " << endl;
    int cost = 0;
    int makespan = 0;
    for (const auto& s : solution)
    {
      cost += s.cost;
      makespan = max<int>(makespan, s.cost);
    }

    ofstream fout(outputFile);
    fout << "statistics:" << endl;
    fout << "  cost: " << cost << endl;
    fout << "  makespan: " << makespan << endl;
    fout << "  runtime: " << timer.elapsedSeconds() << endl;
    cerr << "  runtime: " << timer.elapsedSeconds() << endl;
    fout << "  highLevelExpanded: " << mapf.highLevelExpanded() << endl;
    fout << "  lowLevelExpanded: " << mapf.lowLevelExpanded() << endl;
    fout << "schedule:" << endl;
    for (size_t a = 0; a < solution.size(); ++a)
    {
      // cout << "Solution for: " << a << endl;
      // for (size_t i = 0; i < solution[a].actions.size(); ++i) {
      //   cout << solution[a].states[i].second << ": " <<
      //   solution[a].states[i].first << "->" << solution[a].actions[i].first
      //   << "(cost: " << solution[a].actions[i].second << ")" << endl;
      // }
      // cout << solution[a].states.back().second << ": " <<
      // solution[a].states.back().first << endl;

      fout << "  agent" << a << ":" << endl;
      for (const auto& state : solution[a].states)
      {
        fout << "    - x: " << state.first.x << endl
            << "      y: " << state.first.y << endl
            << "      t: " << state.second << endl;
      }
    }
  }
  else
  {
    cout << "Planning NOT successful!" << endl;
  }

  return 0;
}
