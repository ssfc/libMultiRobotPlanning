//
// Created by take_ on 2023/11/7.
//

#include <fstream>
#include <iostream>

#include <boost/functional/hash.hpp>
#include <boost/program_options.hpp>

#include <yaml-cpp/yaml.h>

#include <libMultiRobotPlanning/cbs_isolated.hpp>
#include "timer.hpp"

using namespace std;
using libMultiRobotPlanning::Neighbor;
using libMultiRobotPlanning::PlanResult;


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


///


int main(int argc, char* argv[])
{
    namespace po = boost::program_options;
    // Declare the supported options.
    po::options_description desc("Allowed options");
    string inputFile;
    string outputFile;
    bool is_disappear_at_goal;
    desc.add_options()("help", "produce help message")
            ("input,i", po::value<string>(&inputFile)->required(), "input file (YAML)")
            ("output,o", po::value<string>(&outputFile)->required(), "output file (YAML)")
            ("disappear-at-goal", po::bool_switch(&is_disappear_at_goal), "make agents to disappear at goal rather than staying there");

    try
    {
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
    vector<TimeLocation> startStates;

    const auto& dim = config["map"]["dimensions"];
    int dimx = dim[0].as<int>();
    int dimy = dim[1].as<int>();

    for (const auto& node : config["map"]["obstacles"])
    {
        obstacles.insert(Location(node[0].as<int>(), node[1].as<int>()));
    }

    for (const auto& node : config["agents"])
    {
        const auto& start = node["start"];
        const auto& goal = node["goal"];
        startStates.emplace_back(TimeLocation(0, start[0].as<int>(), start[1].as<int>()));
        // cout << "s: " << startStates.back() << endl;
        goals.emplace_back(Location(goal[0].as<int>(), goal[1].as<int>()));
    }

    // sanity check: no identical start locations
    unordered_set<TimeLocation> startStatesSet;
    for (const auto& s : startStates)
    {
        if (startStatesSet.find(s) != startStatesSet.end())
        {
            cout << "Identical start locations detected -> no solution!" << endl;

            return 0;
        }

        startStatesSet.insert(s);
    }

    Environment mapf(dimx, dimy, obstacles, goals, is_disappear_at_goal);
    CBS cbs(mapf);
    vector<PlanResult<TimeLocation, Action, int> > solution;

    Timer timer;
    bool is_success = cbs.high_level_search(startStates, solution);
    timer.stop();

    if (is_success)
    {
        cout << "Planning successful! " << endl;
        // cout << "hello" << endl;
        int cost = 0;
        int makespan = 0;
        for (const auto& s : solution)
        {
            cost += s.cost;
            makespan = max<int>(makespan, s.cost);
        }

        ofstream fout(outputFile);
        fout << "statistics:" << endl;

        fout << "cost: " << cost << endl;
        cerr << "cost: " << cost << endl;

        fout << "makespan: " << makespan << endl;

        fout << "runtime: " << timer.elapsedSeconds() << endl;
        cerr << "runtime: " << timer.elapsedSeconds() * 1000 << "ms" << endl;

        fout << "highLevelExpanded: " << mapf.highLevelExpanded() << endl;
        cerr << "highLevelExpanded: " << mapf.highLevelExpanded() << endl;

        fout << "lowLevelExpanded: " << mapf.lowLevelExpanded() << endl;
        cerr << "lowLevelExpanded: " << mapf.lowLevelExpanded() << endl;

        fout << "schedule:" << endl;
        for (size_t a = 0; a < solution.size(); ++a)
        {
            // cout << "Solution for: " << a << endl;
            // for (size_t i = 0; i < solution[a].actions.size(); ++i) {
            //   cout << solution[a].path[i].second << ": " <<
            //   solution[a].path[i].first << "->" << solution[a].actions[i].first
            //   << "(cost: " << solution[a].actions[i].second << ")" << endl;
            // }
            // cout << solution[a].path.back().second << ": " <<
            // solution[a].path.back().first << endl;

            fout << "  agent" << a << ":" << endl;
            for (const auto& state : solution[a].path)
            {
                fout << "    - x: " << state.first.x << endl
                     << "      y: " << state.first.y << endl
                     << "      t: " << state.second << endl;
            }

            cerr << "agent " << a << ": ";
            for (const auto& state : solution[a].path)
            {
                cerr << "(" << state.first.x << "," << state.first.y << "),";
            }
            cerr << endl;
        }
    }
    else
    {
        cout << "Planning NOT successful!" << endl;
    }

    return 0;
}

// Test on ubuntu platform:
// mkdir build; cd build
// cmake .. ; make
// ./cbs -i ../benchmark/8x8_obst12/map_8by8_obst12_agents5_ex_test.yaml -o output.yaml