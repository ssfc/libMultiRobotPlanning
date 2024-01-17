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


int main(int argc, char* argv[])
{
    namespace po = boost::program_options;
    // Declare the supported options.
    po::options_description desc("Allowed options");
    std::string inputFile;
    std::string outputFile;
    desc.add_options()("help", "produce help message")
        ("input,i", po::value<std::string>(&inputFile)->required(), "input file (YAML)")
        ("output,o", po::value<std::string>(&outputFile)->required(), "output file (YAML)")
        // ("url",
        // po::value<std::string>(&url)->default_value("http://0.0.0.0:8080"),
        // "server URL")
        ;

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

    // Configure SIPP based on config file
    YAML::Node config = YAML::LoadFile(inputFile);

    Location goal(config["goal"][0].as<int>(), config["goal"][1].as<int>());
    Location start(config["start"][0].as<int>(), config["start"][1].as<int>());

    std::unordered_set<Location> obstacles;
    for (const auto& node : config["environment"]["obstacles"])
    {
        obstacles.insert(Location(node[0].as<int>(), node[1].as<int>()));
    }

    SIPP sipp(config["environment"]["size"][0].as<int>(), config["environment"]["size"][1].as<int>(),
        obstacles, goal);
    for (const auto& node : config["environment"]["collisionIntervals"])
    {
        Location state(node["location"][0].as<int>(), node["location"][1].as<int>());

        std::vector<Interval> collision_intervals;

        for (const auto& interval : node["intervals"])
        {
            collision_intervals.emplace_back(Interval(interval[0].as<int>(), interval[1].as<int>()));
        }

        sipp.generate_location_to_safe_intervals(state, collision_intervals);
    }

    // Plan
    PlanResult solution;
    bool is_success = sipp.sipp_search(start, Action::Wait, solution);

    if (is_success)
    {
        std::cout << "Planning successful! Total cost: " << solution.cost
                  << std::endl;
        for (size_t i = 0; i < solution.actions.size(); ++i)
        {
            std::cout << solution.path[i].second << ": " << solution.path[i].first
                      << "->" << solution.actions[i].first
                      << "(cost: " << solution.actions[i].second << ")" << std::endl;
        }

        std::cout << solution.path.back().second << ": "
                  << solution.path.back().first << std::endl;

        std::ofstream out(outputFile);
        out << "schedule:" << std::endl;
        out << "  agent1:" << std::endl;
        for (size_t i = 0; i < solution.path.size(); ++i)
        {
            out << "    - x: " << solution.path[i].first.x << std::endl
                << "      y: " << solution.path[i].first.y << std::endl
                << "      t: " << solution.path[i].second << std::endl;
        }
    }
    else
    {
        std::cout << "Planning NOT successful!" << std::endl;
    }

    return 0;
}

// (3) debug on ubuntu by cmake:
// cmake .. ; make
// ./sipp_isolated -i ../test/sipp_1.yaml -o output.yaml