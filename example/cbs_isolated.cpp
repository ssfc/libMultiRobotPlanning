//
// Created by take_ on 2023/11/7.
//

#include <fstream>
#include <iostream>

#include <boost/program_options.hpp>
#include <yaml-cpp/yaml.h>

#include <libMultiRobotPlanning/cbs_isolated.hpp>

using namespace std;
namespace fs = std::filesystem;

ostream& operator<<(ostream& os, const Action& this_action)
{
    switch (this_action)
    {
        case Action::North:
            os << "North";
            break;
        case Action::South:
            os << "South";
            break;
        case Action::East:
            os << "East";
            break;
        case Action::West:
            os << "West";
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
    string input_filename;
    string output_filename;
    bool is_disappear_at_goal;
    desc.add_options()("help", "produce help message")
            ("input,i", po::value<string>(&input_filename)->required(), "input file (YAML)")
            ("output,o", po::value<string>(&output_filename)->required(), "output file (YAML)")
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

    YAML::Node config = YAML::LoadFile(input_filename);

    unordered_set<Location> obstacles;
    vector<Location> goals;
    vector<TimeLocation> start_time_locations;

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
        start_time_locations.emplace_back(TimeLocation(0, Location(start[0].as<int>(), start[1].as<int>())));
        // cout << "s: " << start_time_locations.back() << endl;
        goals.emplace_back(Location(goal[0].as<int>(), goal[1].as<int>()));
    }

    // sanity check: no identical start locations
    unordered_set<TimeLocation> start_time_location_set;
    for (const auto& s : start_time_locations)
    {
        if (start_time_location_set.find(s) != start_time_location_set.end())
        {
            cout << "Identical start locations detected -> no solution!" << endl;

            return 0;
        }

        start_time_location_set.insert(s);
    }

    CBS mapf(dimx, dimy, obstacles, goals.size(), start_time_locations, goals, is_disappear_at_goal);
    // mapf.generate_text_instance("hello.txt");

    bool is_success = mapf.high_level_search();
    if (!is_success)
    {
        cout << "Planning NOT successful!" << endl;
    }

    /*
    std::string yaml_path = "/home/ssfc/libMultiRobotPlanning/benchmark/32x32_obst204";
    std::string txt_path = "/home/ssfc/MAPF-CBS-cpp-mine/benchmark/32x32_obst204";

    // 遍历目录
    for (const auto& entry : fs::directory_iterator(yaml_path))
    {
        // 输出文件名
        std::cout << entry.path().string() << std::endl;
        std::string yaml_filename = entry.path().string();
        size_t pos = entry.path().filename().string().find(".yaml");
        std::string txt_filename = txt_path + "/" + entry.path().filename().string().replace(pos, 5, ".txt");
        std::cerr << txt_filename << std::endl;

        YAML::Node config = YAML::LoadFile(yaml_filename);

        unordered_set<Location> obstacles;
        vector<Location> goals;
        vector<TimeLocation> start_time_locations;

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
            start_time_locations.emplace_back(TimeLocation(0, Location(start[0].as<int>(), start[1].as<int>())));
            // cout << "s: " << start_time_locations.back() << endl;
            goals.emplace_back(Location(goal[0].as<int>(), goal[1].as<int>()));
        }

        CBS mapf(dimx, dimy, obstacles, goals.size(), start_time_locations, goals, is_disappear_at_goal);
        mapf.generate_text_instance(txt_filename);

    }
     //*/

    return 0;
}

// Test on ubuntu platform:
// mkdir build; cd build
// cmake .. ; make
// ./cbs_isolated -i ../benchmark/8x8_obst12/map_8by8_obst12_agents5_ex_test.yaml -o output.yaml
// ./cbs_isolated -i ../benchmark/8x8_obst12/map_8by8_obst12_agents8_ex10.yaml -o output.yaml