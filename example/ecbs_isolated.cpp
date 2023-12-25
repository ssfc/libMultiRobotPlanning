//
// Created by take_ on 2023/12/21.
//

#include <fstream>
#include <iostream>

#include <boost/functional/hash.hpp>
#include <boost/program_options.hpp>

#include <yaml-cpp/yaml.h>

#include <libMultiRobotPlanning/ecbs_isolated.hpp>
#include "timer.hpp"



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


///


int main(int argc, char* argv[])
{
    namespace po = boost::program_options;
    // Declare the supported options.
    po::options_description desc("Allowed options");
    std::string inputFile;
    std::string outputFile;
    bool disappearAtGoal;
    float w;
    desc.add_options()("help", "produce help message")
            ("input,i", po::value<std::string>(&inputFile)->required(), "input file (YAML)")
            ("output,o", po::value<std::string>(&outputFile)->required(), "output file (YAML)")
            ("suboptimality,w", po::value<float>(&w)->default_value(1.0), "suboptimality bound")
            ("disappear-at-goal", po::bool_switch(&disappearAtGoal), "make agents to disappear at goal rather than staying there");

    try
    {
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
    std::vector<TimeLocation> startStates;

    const auto& dim = config["map"]["dimensions"];
    int dimx = dim[0].as<int>();
    int dimy = dim[1].as<int>();

    for (const auto& node : config["map"]["obstacles"]) {
        obstacles.insert(Location(node[0].as<int>(), node[1].as<int>()));
    }

    for (const auto& node : config["agents"]) {
        const auto& start = node["start"];
        const auto& goal = node["goal"];
        startStates.emplace_back(TimeLocation(0, Location(start[0].as<int>(), start[1].as<int>())));
        // std::cout << "s: " << startStates.back() << std::endl;
        goals.emplace_back(Location(goal[0].as<int>(), goal[1].as<int>()));
    }

    // sanity check: no identical start locations
    std::unordered_set<TimeLocation> start_time_location_set;
    for (const auto& s : startStates)
    {
        if (start_time_location_set.find(s) != start_time_location_set.end())
        {
            std::cout << "Identical start locations detected -> no solution!" << std::endl;
            
            return 0;
        }
        start_time_location_set.insert(s);
    }

    ECBSEnvironment mapf(dimx, dimy, obstacles, goals, disappearAtGoal, w);
    ECBS ecbs(mapf, w);
    std::vector<PlanResult> solution;

    Timer timer;
    bool success = ecbs.high_level_search(startStates, solution);
    timer.stop();

    if (success)
    {
        std::cout << "Planning successful! " << std::endl;
        int cost = 0;
        int makespan = 0;

        for (const auto& s : solution)
        {
            cost += s.cost;
            makespan = std::max<int>(makespan, s.cost);
        }

        std::ofstream out(outputFile);
        out << "statistics:" << std::endl;
        out << "  cost: " << cost << std::endl;
        std::cout << "  cost: " << cost << std::endl;

        out << "  makespan: " << makespan << std::endl;
        out << "  runtime: " << timer.elapsedSeconds() << std::endl;
        std::cout << "  runtime: " << timer.elapsedSeconds() << std::endl;

        out << "  highLevelExpanded: " << mapf.highLevelExpanded() << std::endl;
        std::cout << "  highLevelExpanded: " << mapf.highLevelExpanded() << std::endl;

        out << "  lowLevelExpanded: " << mapf.lowLevelExpanded() << std::endl;
        std::cout << "  lowLevelExpanded: " << mapf.lowLevelExpanded() << std::endl;

        out << "schedule:" << std::endl;
        for (size_t a = 0; a < solution.size(); ++a)
        {
            // std::cout << "Solution for: " << a << std::endl;
            // for (size_t i = 0; i < solution[a].actions.size(); ++i) {
            //   std::cout << solution[a].path[i].second << ": " <<
            //   solution[a].path[i].first << "->" << solution[a].actions[i].first
            //   << "(cost: " << solution[a].actions[i].second << ")" << std::endl;
            // }
            // std::cout << solution[a].path.back().second << ": " <<
            // solution[a].path.back().first << std::endl;

            out << "  agent" << a << ":" << std::endl;
            for (const auto& state : solution[a].path)
            {
                out << "    - x: " << state.first.location.x << std::endl
                    << "      y: " << state.first.location.y << std::endl
                    << "      t: " << state.second << std::endl;
            }

            std::cout << "agent " << a << ": ";

            for (const auto& state : solution[a].path)
            {
                std::cout << "(" << state.first.location.x << "," << state.first.location.y << "),";
            }

            std::cout << std::endl;
        }
    }
    else
    {
        std::cout << "Planning NOT successful!" << std::endl;
    }

    return 0;
}
