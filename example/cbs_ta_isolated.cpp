//
// Created by take_ on 2024/2/19.
//


#include "libMultiRobotPlanning/cbs_ta_isolated.hpp"



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
    size_t maxTaskAssignments;
    desc.add_options()("help", "produce help message")
        ("input,i", po::value<std::string>(&inputFile)->required(), "input file (YAML)")
        ("output,o", po::value<std::string>(&outputFile)->required(), "output file (YAML)")
        ("maxTaskAssignments", po::value<size_t>(&maxTaskAssignments)->default_value(1e9),
        "maximum number of task assignments to try");

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

    YAML::Node config = YAML::LoadFile(inputFile);

    std::unordered_set<Location> obstacles;
    std::vector<std::unordered_set<Location> > goals;
    std::vector<State> startStates;

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
        startStates.emplace_back(State(0, start[0].as<int>(), start[1].as<int>()));
        goals.resize(goals.size() + 1);
        for (const auto& goal : node["potentialGoals"])
        {
            goals.back().emplace(Location(goal[0].as<int>(), goal[1].as<int>()));
        }
    }

    // sanity check: no identical start locations
    std::unordered_set<State> startStatesSet;
    for (const auto& s : startStates)
    {
        if (startStatesSet.find(s) != startStatesSet.end())
        {
            std::cout << "Identical start locations detected -> no solution!" << std::endl;

            return 0;
        }

        startStatesSet.insert(s);
    }

    Environment mapf(dimx, dimy, obstacles, startStates, goals, maxTaskAssignments);
 

    CBSTA cbsta(mapf);
    std::vector<PlanResult> solution;

    Timer timer;
    bool success = cbsta.cbsta_search(startStates, solution);
    timer.stop();

    if (success)
    {
        std::cout << "Planning successful! " << std::endl;
        int64_t cost = 0;
        int64_t makespan = 0;
        for (const auto& s : solution)
        {
            cost += s.cost;
            makespan = std::max<int64_t>(makespan, s.cost);
        }

        std::ofstream out(outputFile);
        out << "statistics:" << std::endl;
        out << "  cost: " << cost << std::endl;
        out << "  makespan: " << makespan << std::endl;
        out << "  runtime: " << timer.elapsedSeconds() << std::endl;
        out << "  highLevelExpanded: " << mapf.highLevelExpanded() << std::endl;
        out << "  lowLevelExpanded: " << mapf.lowLevelExpanded() << std::endl;
        out << "  numTaskAssignments: " << mapf.numTaskAssignments() << std::endl;
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
                out << "    - x: " << state.first.x << std::endl
                    << "      y: " << state.first.y << std::endl
                    << "      t: " << state.second << std::endl;
            }
        }
    }
    else
    {
        std::cout << "Planning NOT successful!" << std::endl;
    }

    return 0;
}


// Test on ubuntu platform:
// mkdir build; cd build
// cmake .. ; make
// ./cbs_ta -i ../test/mapfta_simple1_a1.yaml -o output.yaml
// ./cbs_ta -i ../test/mapfta_simple1_a2.yaml -o output.yaml
// ./cbs_ta -i ../test/mapfta_simple1_a3.yaml -o output.yaml