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

    int _num_columns;
    int _num_rows;
    std::unordered_set<Location> _obstacles;
    std::vector<Location> _goals;
    std::vector<TimeLocation> _start_states;

    YAML::Node config = YAML::LoadFile(inputFile);

    /*
    std::ifstream fromfile(inputFile);
    if (fromfile.is_open())
    {
        fromfile >> num_rows >> num_columns;

        std::vector<std::vector<int>> map;
        map.resize(num_rows);
        for (int i = 0; i < num_rows; i++) {
            map[i].resize(num_columns);
        }

        for (int i = 0; i < num_rows; i++)
        {
            for (int j = 0; j < num_columns; j++)
            {
                char c;
                fromfile >> c;
                if (c == '@')
                {
                    map[i][j] = 0;
                    _obstacles.insert(Location(j, i));
                }
                else if (c == '.') {
                    map[i][j] = 1;
                }
            }
        }

        fromfile >> num_agents;
        start_time_locations.resize(num_agents);
        _goals.resize(num_agents);
        for (int i = 0; i < num_agents; i++)
        {
            fromfile >> start_time_locations[i].location.x;
            fromfile >> start_time_locations[i].location.y;
            fromfile >> _goals[i].x;
            fromfile >> _goals[i].y;
        }

        fromfile.close();
        */

    const auto& dim = config["map"]["dimensions"];
    _num_columns = dim[0].as<int>();
    _num_rows = dim[1].as<int>();

    for (const auto& node : config["map"]["_obstacles"]) {
        _obstacles.insert(Location(node[0].as<int>(), node[1].as<int>()));
    }

    for (const auto& node : config["agents"]) {
        const auto& start = node["start"];
        const auto& goal = node["goal"];
        _start_states.emplace_back(TimeLocation(0, Location(start[0].as<int>(), start[1].as<int>())));
        // std::cout << "s: " << _start_states.back() << std::endl;
        _goals.emplace_back(Location(goal[0].as<int>(), goal[1].as<int>()));
    }

    // sanity check: no identical start locations
    std::unordered_set<TimeLocation> start_time_location_set;
    for (const auto& _state : _start_states)
    {
        if (start_time_location_set.find(_state) != start_time_location_set.end())
        {
            std::cout << "Identical start locations detected -> no solution!" << std::endl;

            return 0;
        }

        start_time_location_set.insert(_state);
    }

    ECBS mapf(_num_columns, _num_rows, _obstacles, _goals, disappearAtGoal, w);
    std::vector<PlanResult> solution;

    Timer timer;
    bool success = mapf.high_level_search(_start_states, solution);
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

        out << "  highLevelExpanded: " << mapf.get_num_expanded_high_level_nodes() << std::endl;
        std::cout << "  highLevelExpanded: " << mapf.get_num_expanded_high_level_nodes() << std::endl;

        out << "  lowLevelExpanded: " << mapf.get_num_expanded_low_level_nodes() << std::endl;
        std::cout << "  lowLevelExpanded: " << mapf.get_num_expanded_low_level_nodes() << std::endl;

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

// (3) debug on ubuntu by cmake:
// cmake .. ; make
// ./ecbs_isolated -i ../benchmark/8x8_obst12/map_8by8_obst12_agents8_ex10.yaml -o output.yaml -w 1
// ./ecbs_isolated -i ../benchmark/8x8_obst12/map_8by8_obst12_agents8_ex10.yaml -o output.yaml -w 1.5