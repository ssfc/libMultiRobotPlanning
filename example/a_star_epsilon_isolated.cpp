//
// Created by take_ on 2023/11/25.
//

#include <fstream>
#include <iostream>

#include <boost/functional/hash.hpp>
#include <boost/program_options.hpp>

#include <libMultiRobotPlanning/a_star_epsilon_isolated.hpp>



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
    Environment(size_t dimx, size_t dimy, std::unordered_set<Location> obstacles,
                Location goal)
            : num_columns(dimx),
              num_rows(dimy),
              obstacles(std::move(obstacles)),
              m_goal(std::move(goal))  // NOLINT
    {}

    int admissible_heuristic(const Location& s) {
        return std::abs(s.x - m_goal.x) + std::abs(s.y - m_goal.y);
    }

    // a potentially inadmissible heuristic
    int focalStateHeuristic(const Location& /*s*/, int gScore) {
        // prefer lower g-values
        return gScore;
    }

    int focalTransitionHeuristic(const Location& /*s1*/, const Location& /*s2*/,
                                 int gScoreS1, int gScoreS2) {
        // prefer lower g-values
        return gScoreS2 - gScoreS1;
    }

    bool is_solution(const Location& s) { return s == m_goal; }

    void get_neighbors(const Location& s, std::vector<Child>& neighbors)
   {
        neighbors.clear();

        Location up(s.x, s.y + 1);
        if (location_valid(up))
        {
            neighbors.emplace_back(Child(up, Action::Up, 1));
        }

        Location down(s.x, s.y - 1);
        if (location_valid(down))
        {
            neighbors.emplace_back(Child(down, Action::Down, 1));
        }

        Location left(s.x - 1, s.y);
        if (location_valid(left))
        {
            neighbors.emplace_back(Child(left, Action::Left, 1));
        }

        Location right(s.x + 1, s.y);
        if (location_valid(right))
        {
            neighbors.emplace_back(Child(right, Action::Right, 1));
        }
    }

    void onExpandNode(const Location& /*s*/, int /*fScore*/, int /*gScore*/) {}

    void onDiscover(const Location& /*s*/, int /*fScore*/, int /*gScore*/) {}

public:
    bool location_valid(const Location& s) {
        return s.x >= 0 && s.x < num_columns && s.y >= 0 && s.y < num_rows &&
               obstacles.find(s) == obstacles.end();
    }

private:
    int num_columns;
    int num_rows;
    std::unordered_set<Location> obstacles;
    Location m_goal;
};

int main(int argc, char* argv[])
{
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

    std::unordered_set<Location> obstacles;

    std::ifstream map(mapFile);
    int dimX = 0;
    int y = 0;
    while (map.good()) {
        std::string line;
        std::getline(map, line);
        int x = 0;
        for (char c : line) {
            if (c == '@') {
                obstacles.insert(Location(x, y));
            }
            ++x;
        }
        dimX = std::max(dimX, x);
        ++y;
    }
    std::cout << dimX << " " << y << std::endl;

    bool is_success = false;

    Location goal(goalX, goalY);
    Location start(startX, startY);
    Environment env(dimX, y - 1, obstacles, goal);

    AStarEpsilon<Location, Action, int, Environment> astar(env, w);

    AgentPlan<int> solution;

    if (env.location_valid(start)) {
        is_success = astar.search(start, solution);
    }

    std::ofstream out(outputFile);
    if (is_success) {
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



