#include <fstream>
#include <iostream>

#include <boost/functional/hash.hpp>
#include <boost/program_options.hpp>

#include <libMultiRobotPlanning/a_star_epsilon.hpp>

using libMultiRobotPlanning::AStarEpsilon;
using libMultiRobotPlanning::Neighbor;
using libMultiRobotPlanning::PlanResult;

struct State {
  State(int x, int y) : x(x), y(y) {}

  State(const State&) = default;
  State(State&&) = default;
  State& operator=(const State&) = default;
  State& operator=(State&&) = default;

  bool operator==(const State& other) const {
    return std::tie(x, y) == std::tie(other.x, other.y);
  }

  friend std::ostream& operator<<(std::ostream& os, const State& s) {
    return os << "(" << s.x << "," << s.y << ")";
  }

  int x;
  int y;
};

namespace std {
template <>
struct hash<State> {
  size_t operator()(const State& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.x);
    boost::hash_combine(seed, s.y);
    return seed;
  }
};
}  // namespace std

enum class Action {
  Up,
  Down,
  Left,
  Right,
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
  }
  return os;
}

class Environment {
 public:
  Environment(size_t dimx, size_t dimy, std::unordered_set<State> obstacles,
              State goal)
      : num_columns(dimx),
        num_rows(dimy),
        obstacles(std::move(obstacles)),
        m_goal(std::move(goal))  // NOLINT
  {}

  int admissible_heuristic(const State& s) {
    return std::abs(s.x - m_goal.x) + std::abs(s.y - m_goal.y);
  }

  // a potentially inadmissible heuristic
  int focalStateHeuristic(const State& /*s*/, int gScore) {
    // prefer lower g-values
    return gScore;
  }

  int focalTransitionHeuristic(const State& /*s1*/, const State& /*s2*/,
                               int gScoreS1, int gScoreS2) {
    // prefer lower g-values
    return gScoreS2 - gScoreS1;
  }

  bool is_solution(const State& s) { return s == m_goal; }

  void get_neighbors(const State& s,
                    std::vector<Neighbor<State, Action, int> >& neighbors) {
    neighbors.clear();

    State up(s.x, s.y + 1);
    if (location_valid(up)) {
      neighbors.emplace_back(Neighbor<State, Action, int>(up, Action::Up, 1));
    }
    State down(s.x, s.y - 1);
    if (location_valid(down)) {
      neighbors.emplace_back(
          Neighbor<State, Action, int>(down, Action::Down, 1));
    }
    State left(s.x - 1, s.y);
    if (location_valid(left)) {
      neighbors.emplace_back(
          Neighbor<State, Action, int>(left, Action::Left, 1));
    }
    State right(s.x + 1, s.y);
    if (location_valid(right)) {
      neighbors.emplace_back(
          Neighbor<State, Action, int>(right, Action::Right, 1));
    }
  }

  void onExpandNode(const State& /*s*/, int /*fScore*/, int /*gScore*/) {}

  void onDiscover(const State& /*s*/, int /*fScore*/, int /*gScore*/) {}

 public:
  bool location_valid(const State& s) {
    return s.x >= 0 && s.x < num_columns && s.y >= 0 && s.y < num_rows &&
           obstacles.find(s) == obstacles.end();
  }

 private:
  int num_columns;
  int num_rows;
  std::unordered_set<State> obstacles;
  State m_goal;
};

int main(int argc, char* argv[]) {
  namespace po = boost::program_options;
  // Declare the supported options.
  po::options_description desc("Allowed options");
  int startX, startY, goalX, goalY;
  std::string mapFile;
  std::string outputFile;
  float w;
  desc.add_options()("help", "produce help message")
      ("startX", po::value<int>(&startX)->required(), "start position x-component")
      ("startY", po::value<int>(&startY)->required(), "start position y-component")
      ("goalX", po::value<int>(&goalX)->required(), "goal position x-component")
      ("goalY", po::value<int>(&goalY)->required(), "goal position y-component")
      ("map,m", po::value<std::string>(&mapFile)->required(), "input map (txt)")
      ("output,o", po::value<std::string>(&outputFile)->required(), "output file (YAML)")
      ("suboptimality,w", po::value<float>(&w)->required(), "suboptimality factor");

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

  std::unordered_set<State> obstacles;

  std::ifstream map(mapFile);
  int dimX = 0;
  int y = 0;
  while (map.good()) {
    std::string line;
    std::getline(map, line);
    int x = 0;
    for (char c : line) {
      if (c == '#') {
        obstacles.insert(State(x, y));
      }
      ++x;
    }
    dimX = std::max(dimX, x);
    ++y;
  }
  std::cout << dimX << " " << y << std::endl;

  bool success = false;

  State goal(goalX, goalY);
  State start(startX, startY);
  Environment env(dimX, y - 1, obstacles, goal);

  AStarEpsilon<State, Action, int, Environment> astar(env, w);

  PlanResult<State, Action, int> solution;

  if (env.location_valid(start)) {
    success = astar.search(start, solution);
  }

  std::ofstream out(outputFile);
  if (success) {
    std::cout << "Planning successful! Total cost: " << solution.cost
              << std::endl;
    for (size_t i = 0; i < solution.actions.size(); ++i) {
      std::cout << solution.path[i].second << ": " << solution.path[i].first
                << "->" << solution.actions[i].first
                << "(cost: " << solution.actions[i].second << ")" << std::endl;
    }
    std::cout << solution.path.back().second << ": "
              << solution.path.back().first << std::endl;

    out << "schedule:" << std::endl;
    out << "  agent1:" << std::endl;
    for (size_t i = 0; i < solution.path.size(); ++i) {
      out << "    - x: " << solution.path[i].first.x << std::endl
          << "      y: " << solution.path[i].first.y << std::endl
          << "      t: " << i << std::endl;
    }
  } else {
    std::cout << "Planning NOT successful!" << std::endl;
  }

  return 0;
}
