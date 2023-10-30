#include <fstream>
#include <iostream>

#include <boost/functional/hash.hpp>
#include <boost/program_options.hpp>

#include <libMultiRobotPlanning/a_star.hpp>

using namespace std;
using libMultiRobotPlanning::AStar;
using libMultiRobotPlanning::Neighbor;
using libMultiRobotPlanning::PlanResult;

class State
{
public:
    int x;
    int y;

public:
    State(int x, int y) : x(x), y(y) {}

    State(const State&) = default;
    State(State&&) = default;
    State& operator=(const State&) = default;
    State& operator=(State&&) = default;

    bool operator==(const State& other) const
    {
        return tie(x, y) == tie(other.x, other.y);
    }

    friend ostream& operator<<(ostream& os, const State& s)
    {
        return os << "(" << s.x << "," << s.y << ")";
    }
};

namespace std
{
    template <>
    struct hash<State>
    {
        size_t operator()(const State& s) const
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
    int m_dimx;
    int m_dimy;
    unordered_set<State> m_obstacles;
    State m_goal;

public:
    Environment(size_t dimx, size_t dimy, unordered_set<State> obstacles, State goal)
    : m_dimx(dimx),
      m_dimy(dimy),
      m_obstacles(move(obstacles)),
      m_goal(std::move(goal))  // NOLINT
    {}

    int admissibleHeuristic(const State& s)
    {
        return abs(s.x - m_goal.x) + abs(s.y - m_goal.y);
    }

    bool isSolution(const State& s)
    {
        return s == m_goal;
    }

    void getNeighbors(const State& s, vector<Neighbor<State, Action, int> >& neighbors)
    {
        neighbors.clear();

        State up(s.x, s.y + 1);

        if (stateValid(up))
        {
            neighbors.emplace_back(Neighbor<State, Action, int>(up, Action::Up, 1));
        }

        State down(s.x, s.y - 1);

        if (stateValid(down))
        {
            neighbors.emplace_back(Neighbor<State, Action, int>(down, Action::Down, 1));
        }

        State left(s.x - 1, s.y);

        if (stateValid(left))
        {
            neighbors.emplace_back(Neighbor<State, Action, int>(left, Action::Left, 1));
        }

        State right(s.x + 1, s.y);

        if (stateValid(right))
        {
            neighbors.emplace_back(Neighbor<State, Action, int>(right, Action::Right, 1));
        }
    }

    void onExpandNode(const State& /*s*/, int /*fScore*/, int /*gScore*/) {}

    void onDiscover(const State& /*s*/, int /*fScore*/, int /*gScore*/) {}

    bool stateValid(const State& s)
    {
        return s.x >= 0 && s.x < m_dimx && s.y >= 0 && s.y < m_dimy &&
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

    unordered_set<State> obstacles;

    ifstream map(mapFile);
    int dimX = 0;
    int y = 0;
    while (map.good())
    {
        string line;
        getline(map, line);
        int x = 0;
        for (char c : line)
        {
            if (c == '#')  // # 代表障碍物, . 代表空间。
            {
                obstacles.insert(State(x, y));
            }

            ++x;
        }

        dimX = max(dimX, x);
        ++y;
    }

    cout << dimX << " " << y << endl;

    bool success = false;

    State goal(goalX, goalY);
    State start(startX, startY);
    Environment env(dimX, y - 1, obstacles, goal);

    AStar<State, Action, int, Environment> astar(env);

    PlanResult<State, Action, int> solution;

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
