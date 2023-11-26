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

    try
    {
        po::variables_map vm;
        po::store(po::parse_command_line(argc, argv, desc), vm);
        po::notify(vm);

        if (vm.count("help") != 0u)
        {
            std::cout << desc << "\n";

            return 0;
        }
    }
    catch (po::error& e)
    {
        std::cerr << e.what() << std::endl << std::endl;
        std::cerr << desc << std::endl;

        return 1;
    }

    std::unordered_set<Location> obstacles;

    std::ifstream map(mapFile);
    int dimX = 0;
    int y = 0;
    while (map.good())
    {
        std::string line;
        std::getline(map, line);
        int x = 0;
        for (char c : line)
        {
            if (c == '@')
            {
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
    Environment env(dimX, y - 1, obstacles, goal, w);

    AStarEpsilon astar(env, w);

    AgentPlan solution;

    if (env.location_valid(start))
    {
        // is_success = astar.a_star_epsilon_search(start, solution);
        is_success = env.a_star_epsilon_search(start, solution);
    }

    std::ofstream out(outputFile);
    if (is_success)
    {
        std::cout << "Planning successful! Total cost: " << solution.cost << std::endl;
        for (size_t i = 0; i < solution.actions.size(); ++i)
        {
            std::cout << solution.path[i].second << ": " << solution.path[i].first
                      << "->" << solution.actions[i].first
                      << "(cost: " << solution.actions[i].second << ")" << std::endl;
        }

        std::cout << solution.path.back().second << ": "
                  << solution.path.back().first << std::endl;

        out << "schedule:" << std::endl;
        out << "  agent1:" << std::endl;

        for (size_t i = 0; i < solution.path.size(); ++i)
        {
            out << "    - x: " << solution.path[i].first.x << std::endl
                << "      y: " << solution.path[i].first.y << std::endl
                << "      t: " << i << std::endl;
        }

    }
    else
    {
        std::cout << "Planning NOT successful!" << std::endl;
    }

    return 0;
}



