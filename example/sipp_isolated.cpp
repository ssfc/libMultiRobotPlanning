//
// Created by take_ on 2024/1/11.
//

#include <fstream>
#include <iostream>

#include <boost/functional/hash.hpp>
#include <boost/program_options.hpp>

#include <yaml-cpp/yaml.h>

#include <libMultiRobotPlanning/sipp_isolated.hpp>


std::ostream& operator<<(std::ostream& os, const Action& a)
{
    switch (a)
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

class Environment
{
private:
    int num_columns;
    int num_rows;
    std::unordered_set<Location> obstacles;
    Location m_goal;

public:
    Environment(size_t dimx, size_t dimy, std::unordered_set<Location> obstacles, Location goal)
        : num_columns(dimx),
          num_rows(dimy),
          obstacles(std::move(obstacles)),
          m_goal(goal)
    {}

    bool location_valid(const Location& s)
    {
        return s.x >= 0 && s.x < num_columns && s.y >= 0 && s.y < num_rows &&
               obstacles.find(s) == obstacles.end();
    }

    float admissible_heuristic(const Location& s)
    {
        return std::abs(s.x - m_goal.x) + std::abs(s.y - m_goal.y);
    }

    bool is_solution(const Location& s)
    {
        return s == m_goal;
    }

    Location getLocation(const Location& s)
    {
        return s;
    }

    void get_neighbors(const Location& s, std::vector<Neighbor<Location, Action, int> >& neighbors)
    {
        neighbors.clear();

        Location up(s.x, s.y + 1);
        if (location_valid(up))
        {
            neighbors.emplace_back(Neighbor<Location, Action, int>(up, Action::Up, 1));
        }

        Location down(s.x, s.y - 1);
        if (location_valid(down))
        {
            neighbors.emplace_back(Neighbor<Location, Action, int>(down, Action::Down, 1));
        }

        Location left(s.x - 1, s.y);
        if (location_valid(left))
        {
            neighbors.emplace_back(Neighbor<Location, Action, int>(left, Action::Left, 1));
        }

        Location right(s.x + 1, s.y);
        if (location_valid(right))
        {
            neighbors.emplace_back(Neighbor<Location, Action, int>(right, Action::Right, 1));
        }
    }

    void onExpandNode(const Location& /*s*/, int /*fScore*/, int /*gScore*/)
    {
        // std::cout << "expand: " << s << "g: " << gScore << std::endl;
    }

    void onDiscover(const Location& /*s*/, int /*fScore*/, int /*gScore*/)
    {
        // std::cout << "  discover: " << s << std::endl;
    }

    bool isCommandValid(
        const Location& /*s1*/, const Location& /*s2*/, const Action& /*a*/,
        int earliestStartTime,      // can start motion at this time
        int /*latestStartTime*/,    // must have left s by this time
        int earliestArrivalTime,    // can only arrive at (s+cmd)
        int /*latestArrivalTime*/,  // needs to arrive by this time at (s+cmd)
        int& t)
    {
        t = std::max<int>(earliestArrivalTime, earliestStartTime + 1);

        // TODO(whoenig): need to check for swaps here...

        // return t - 1 <= latestStartTime;
        return true;
    }
};

int main(int argc, char* argv[]) {
    namespace po = boost::program_options;
    // Declare the supported options.
    po::options_description desc("Allowed options");
    std::string inputFile;
    std::string outputFile;
    desc.add_options()("help", "produce help message")(
        "input,i", po::value<std::string>(&inputFile)->required(),
        "input file (YAML)")("output,o",
                             po::value<std::string>(&outputFile)->required(),
                             "output file (YAML)")
        // ("url",
        // po::value<std::string>(&url)->default_value("http://0.0.0.0:8080"),
        // "server URL")
        ;

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

    // Configure SIPP based on config file
    YAML::Node config = YAML::LoadFile(inputFile);

    Location goal(config["goal"][0].as<int>(), config["goal"][1].as<int>());
    Location start(config["start"][0].as<int>(), config["start"][1].as<int>());

    std::unordered_set<Location> obstacles;
    for (const auto& node : config["environment"]["obstacles"]) {
        obstacles.insert(Location(node[0].as<int>(), node[1].as<int>()));
    }
    Environment env(config["environment"]["size"][0].as<int>(),
                    config["environment"]["size"][1].as<int>(), obstacles, goal);

    typedef SIPP<Location, Location, Action, int, Environment> sipp_t;
    sipp_t sipp(env);

    for (const auto& node : config["environment"]["collisionIntervals"]) {
        Location state(node["location"][0].as<int>(), node["location"][1].as<int>());

        std::vector<sipp_t::interval> collisionIntervals;

        for (const auto& interval : node["intervals"]) {
            collisionIntervals.emplace_back(
                sipp_t::interval(interval[0].as<int>(), interval[1].as<int>()));
        }
        sipp.setCollisionIntervals(state, collisionIntervals);
    }

    // Plan
    PlanResult<Location, Action, int> solution;
    bool success = sipp.search(start, Action::Wait, solution);

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

        std::ofstream out(outputFile);
        out << "schedule:" << std::endl;
        out << "  agent1:" << std::endl;
        for (size_t i = 0; i < solution.path.size(); ++i) {
            out << "    - x: " << solution.path[i].first.x << std::endl
                << "      y: " << solution.path[i].first.y << std::endl
                << "      t: " << solution.path[i].second << std::endl;
        }
    } else {
        std::cout << "Planning NOT successful!" << std::endl;
    }

    return 0;
}
