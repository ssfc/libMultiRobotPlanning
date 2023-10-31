#include <fstream>
#include <iostream>

#include <boost/functional/hash.hpp>
#include <boost/program_options.hpp>

#include <libMultiRobotPlanning/a_star.hpp>

using namespace std;
using libMultiRobotPlanning::AStar;
using libMultiRobotPlanning::Neighbor;
using libMultiRobotPlanning::PlanResult;

class Location
{
public:
    int x;
    int y;

public:
    Location(int x, int y) : x(x), y(y) {}
    Location(const Location&) = default;
    Location(Location&&) = default;
    Location& operator=(const Location&) = default;
    Location& operator=(Location&&) = default;

    bool operator==(const Location& other) const
    {
        return tie(x, y) == tie(other.x, other.y);
    }

    friend ostream& operator<<(ostream& os, const Location& s)
    {
        return os << "(" << s.x << "," << s.y << ")";
    }
};

namespace std
{
    template <>
    struct hash<Location>
    {
        size_t operator()(const Location& s) const
        {
            size_t seed = 0;
            boost::hash_combine(seed, s.x);
            boost::hash_combine(seed, s.y);

            return seed;
        }
    };
}  // namespace std

enum class Action
{
    Up,
    Down,
    Left,
    Right,
};

ostream& operator<<(ostream& os, const Action& a)
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
    }

    return os;
}

class Environment
{
private:
    int num_columns;
    int m_dimy;
    unordered_set<Location> m_obstacles;
    Location goal;

public:
    Environment(size_t dimx, size_t dimy, unordered_set<Location> obstacles, Location goal)
    : num_columns(dimx),
      m_dimy(dimy),
      m_obstacles(move(obstacles)),
      goal(std::move(goal))  // NOLINT
    {}

    int admissible_heuristic(const Location& current_location)
    {
        return abs(current_location.x - goal.x)
        + abs(current_location.y - goal.y);
    }

    bool is_solution(const Location& current_location)
    {
        return current_location == goal;
    }

    void get_neighbors(const Location& s, vector<Neighbor<Location, Action, int> >& neighbors)
    {
        neighbors.clear();

        Location up(s.x, s.y + 1);

        if (stateValid(up))
        {
            neighbors.emplace_back(Neighbor<Location, Action, int>(up, Action::Up, 1));
        }

        Location down(s.x, s.y - 1);

        if (stateValid(down))
        {
            neighbors.emplace_back(Neighbor<Location, Action, int>(down, Action::Down, 1));
        }

        Location left(s.x - 1, s.y);

        if (stateValid(left))
        {
            neighbors.emplace_back(Neighbor<Location, Action, int>(left, Action::Left, 1));
        }

        Location right(s.x + 1, s.y);

        if (stateValid(right))
        {
            neighbors.emplace_back(Neighbor<Location, Action, int>(right, Action::Right, 1));
        }
    }

    void onExpandNode(const Location& /*s*/, int /*fScore*/, int /*gScore*/) {}

    void onDiscover(const Location& /*s*/, int /*fScore*/, int /*gScore*/) {}

    bool stateValid(const Location& s)
    {
        return s.x >= 0 && s.x < num_columns && s.y >= 0 && s.y < m_dimy &&
               m_obstacles.find(s) == m_obstacles.end();
    }
};

int main(int argc, char* argv[])
{
    namespace po = boost::program_options;
    // Declare the supported options.
    po::options_description desc("Allowed options");

    int startX, startY, goalX, goalY;
    string mapFile;
    string outputFile;

    desc.add_options()("help", "produce help message")
    ("startX", po::value<int>(&startX)->required(), "start position x-component")
    ("startY", po::value<int>(&startY)->required(), "start position y-component")
    ("goalX", po::value<int>(&goalX)->required(), "goal position x-component")
    ("goalY", po::value<int>(&goalY)->required(), "goal position y-component")
    ("map,m", po::value<string>(&mapFile)->required(), "input map (txt)")
    ("output,o", po::value<string>(&outputFile)->required(), "output file (YAML)");

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

    unordered_set<Location> obstacles;

    ifstream map(mapFile);
    int dim_x = 0;
    int y = 0;

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

    bool success = false;

    Location goal(goalX, goalY);
    Location start(startX, startY);
    Environment env(dim_x, y - 1, obstacles, goal);

    AStar<Location, Action, int, Environment> astar(env);

    PlanResult<Location, Action, int> solution;

    if (env.stateValid(start))
    {
        success = astar.search(start, solution);
    }

    ofstream out(outputFile);
    if (success)
    {
        cout << "Planning successful! Total cost: " << solution.cost << endl;
        for (size_t i = 0; i < solution.actions.size(); ++i)
        {
            cout << solution.states[i].second << ": " << solution.states[i].first
            << "->" << solution.actions[i].first
            << "(cost: " << solution.actions[i].second << ")" << endl;
        }

        cout << solution.states.back().second << ": "
        << solution.states.back().first << endl;

        out << "schedule:" << endl;
        out << "  agent1:" << endl;

        for (size_t i = 0; i < solution.states.size(); ++i)
        {
            out << "    - x: " << solution.states[i].first.x << endl
              << "      y: " << solution.states[i].first.y << endl
              << "      t: " << i << endl;
        }
    }
    else
    {
        cout << "Planning NOT successful!" << endl;
    }

    return 0;
}
