﻿#include <fstream>
#include <iostream>

#include <boost/functional/hash.hpp>
#include <boost/program_options.hpp>

#include <libMultiRobotPlanning/a_star.hpp>

using namespace std;
using libMultiRobotPlanning::Neighbor;
using libMultiRobotPlanning::PlanResult;




enum class Action // 模板类Action实例化
{
    Up,
    Down,
    Left,
    Right,
};

ostream& operator<<(ostream& os, const Action& action)
{
    switch (action)
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
    }

    return os;
}

class Environment // 模板类Environment实例化
{
private:
    int num_columns;
    int num_rows;
    unordered_set<Location> obstacles;
    Location start;
    Location goal;

public:
    Environment(size_t input_num_columns, size_t input_num_rows,
                unordered_set<Location> input_obstacles, Location input_start, Location input_goal)
    : num_columns(input_num_columns),
      num_rows(input_num_rows),
      obstacles(move(input_obstacles)),
      start(std::move(input_start)),
      goal(std::move(input_goal))  // NOLINT
    {}

    // 估算h_score
    int admissible_heuristic(const Location& current_location)
    {
        return abs(current_location.x - goal.x)
        + abs(current_location.y - goal.y);
    }

    // whether agent reach goal or not.
    bool is_solution(const Location& current_location)
    {
        return current_location == goal;
    }

    // whether location valid or not
    bool location_valid(const Location& location)
    {
        return location.x >= 0 && location.x < num_columns
        && location.y >= 0 && location.y < num_rows
        && obstacles.find(location) == obstacles.end();
    }

    // get neighbor of current location
    void get_neighbors(const Location& location, vector<Neighbor<Location, Action, int> >& neighbors)
    {
        neighbors.clear();

        Location north_neighbor(location.x, location.y + 1);

        if (location_valid(north_neighbor))
        {
            neighbors.emplace_back(Neighbor<Location, Action, int>(north_neighbor, Action::Up, 1));
        }

        Location south_neighbor(location.x, location.y - 1);

        if (location_valid(south_neighbor))
        {
            neighbors.emplace_back(Neighbor<Location, Action, int>(south_neighbor, Action::Down, 1));
        }

        Location west_neighbor(location.x - 1, location.y);

        if (location_valid(west_neighbor))
        {
            neighbors.emplace_back(Neighbor<Location, Action, int>(west_neighbor, Action::Left, 1));
        }

        Location east_neighbor(location.x + 1, location.y);

        if (location_valid(east_neighbor))
        {
            neighbors.emplace_back(Neighbor<Location, Action, int>(east_neighbor, Action::Right, 1));
        }
    }

    void onExpandNode(const Location& /*s*/, int /*fScore*/, int /*gScore*/) {}

    void onDiscover(const Location& /*s*/, int /*fScore*/, int /*gScore*/) {}
};

int main(int argc, char* argv[])
{
    namespace po = boost::program_options;
    // Declare the supported options.
    po::options_description desc("Allowed options");

    int start_x;
    int start_y;
    int goal_x;
    int goal_y;

    string map_file;
    string output_file;

    desc.add_options()("help", "produce help message")
    ("startX", po::value<int>(&start_x)->required(), "start position x-component")
    ("startY", po::value<int>(&start_y)->required(), "start position y-component")
    ("goalX", po::value<int>(&goal_x)->required(), "goal position x-component")
    ("goalY", po::value<int>(&goal_y)->required(), "goal position y-component")
    ("map,m", po::value<string>(&map_file)->required(), "input map (txt)")
    ("output,o", po::value<string>(&output_file)->required(), "output file (YAML)");

    try
    {
        po::variables_map vm;
        // 将命令行参数解析成键-值对，然后存储在之前声明的vm变量中。
        // argc是命令行参数的数量，argv是包含实际参数值的字符串数组，desc是一个用于描述程序可接受的命令行参数的配置对象。
        po::store(po::parse_command_line(argc, argv, desc), vm);
        // 通知boost::program_options库，让它更新相关的变量，以便程序可以使用这些参数的值。
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

    unordered_set<Location> obstacles;

    ifstream map(map_file);
    int dim_x = 0;
    int y = 0;

    // 记录长宽，用unordered_set存储障碍物位置
    while (map.good())
    {
        string line;
        getline(map, line);
        int x = 0;
        for (char c : line)
        {
            if (c == '@')  // @ 代表障碍物, . 代表空间。
            {
                obstacles.insert(Location(x, y));
            }

            ++x; // x是列
        }

        dim_x = max(dim_x, x);
        ++y; // y是行
    }

    // 文中dim_x这个变量是有必要的，因为它用来记录地图的宽度，以便在读取地图文件时判断坐标是否有效。
    // dim_x的值是根据地图文件中每一行的字符数来确定的，所以它可能不等于地图文件的列数。
    // dim_x的最大值是地图文件中最长的一行的字符数。
    cout << dim_x << " " << y << endl;

    bool is_success = false;

    Location test_start(start_x, start_y); // 构造、重载相等、重载输出
    Location test_goal(goal_x, goal_y); // 构造、重载相等、重载输出
    Environment test_environment(dim_x, y - 1, obstacles, test_start, test_goal);
    AStar<Location, Action, Environment> test_astar(test_environment); // 模板类实例化
    PlanResult<Location, Action, int> solution;

    if (test_environment.location_valid(test_start))
    {
        is_success = test_astar.a_star_search(test_start, solution);
    }

    ofstream out(output_file);
    if (is_success)
    {
        cout << "Planning successful! Total cost: " << solution.cost << endl;
        for (size_t i = 0; i < solution.actions.size(); ++i)
        {
            cout << solution.path[i].second << ": " << solution.path[i].first
            << "->" << solution.actions[i].first
            << "(cost: " << solution.actions[i].second << ")" << endl;
        }

        cout << solution.path.back().second << ": "
        << solution.path.back().first << endl;

        out << "schedule:" << endl;
        out << "  agent1:" << endl;

        for (size_t i = 0; i < solution.path.size(); ++i)
        {
            out << "    - x: " << solution.path[i].first.x << endl
              << "      y: " << solution.path[i].first.y << endl
              << "      t: " << i << endl;
        }
    }
    else
    {
        cout << "Planning NOT successful!" << endl;
    }

    return 0;
}
