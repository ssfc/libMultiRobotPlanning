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


// Action Custom action for the search.
// 枚举类
enum class Action
{
    Up,
    Down,
    Left,
    Right,
    Wait,
};

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

// Conflict Custom conflict description.
// A conflict needs to be able to be transformed into a constraint.
class Conflict
{
public:
    enum Type
    {
        Vertex,
        Edge,
    };

    int time;
    size_t agent1;
    size_t agent2;
    Type type;

    int x1;
    int y1;
    int x2;
    int y2;

public:
    friend ostream& operator<<(ostream& os, const Conflict& conflict)
    {
        switch (conflict.type)
        {
            case Vertex:
                return os << conflict.time << ": Vertex(" << conflict.x1 << "," << conflict.y1 << ")";
            case Edge:
                return os << conflict.time << ": Edge(" << conflict.x1 << "," << conflict.y1 << ","
                          << conflict.x2 << "," << conflict.y2 << ")";
        }

        return os;
    }
};

class VertexConstraint
{
public:
    int time;
    int x;
    int y;

public:
    VertexConstraint(int time, int x, int y)
            : time(time), x(x), y(y)
    {}

    bool operator==(const VertexConstraint& other) const
    {
        return tie(time, x, y) == tie(other.time, other.x, other.y);
    }

    bool operator<(const VertexConstraint& other) const
    {
        return tie(time, x, y) < tie(other.time, other.x, other.y);
    }

    friend ostream& operator<<(ostream& os, const VertexConstraint& c)
    {
        return os << "VC(" << c.time << "," << c.x << "," << c.y << ")";
    }
};

namespace std
{
    template <>
    struct hash<VertexConstraint>
    {
        size_t operator()(const VertexConstraint& s) const
        {
            size_t seed = 0;
            boost::hash_combine(seed, s.time);
            boost::hash_combine(seed, s.x);
            boost::hash_combine(seed, s.y);
            return seed;
        }
    };
}

class EdgeConstraint
{
public:
    int time;
    int x1;
    int y1;
    int x2;
    int y2;

public:
    EdgeConstraint(int time, int x1, int y1, int x2, int y2)
            : time(time), x1(x1), y1(y1), x2(x2), y2(y2) {}

    bool operator<(const EdgeConstraint& other) const
    {
        return tie(time, x1, y1, x2, y2) <
               tie(other.time, other.x1, other.y1, other.x2, other.y2);
    }

    bool operator==(const EdgeConstraint& other) const
    {
        return tie(time, x1, y1, x2, y2) ==
               tie(other.time, other.x1, other.y1, other.x2, other.y2);
    }

    friend ostream& operator<<(ostream& os, const EdgeConstraint& c)
    {
        return os << "EC(" << c.time << "," << c.x1 << "," << c.y1 << "," << c.x2
                  << "," << c.y2 << ")";
    }
};

namespace std
{
    template <>
    struct hash<EdgeConstraint>
    {
        size_t operator()(const EdgeConstraint& s) const
        {
            size_t seed = 0;
            boost::hash_combine(seed, s.time);
            boost::hash_combine(seed, s.x1);
            boost::hash_combine(seed, s.y1);
            boost::hash_combine(seed, s.x2);
            boost::hash_combine(seed, s.y2);

            return seed;
        }
    };
}

class Constraints
{
public:
    unordered_set<VertexConstraint> vertexConstraints;
    unordered_set<EdgeConstraint> edgeConstraints;

public:
    void add(const Constraints& other)
    {
        vertexConstraints.insert(other.vertexConstraints.begin(),
                                 other.vertexConstraints.end());
        edgeConstraints.insert(other.edgeConstraints.begin(),
                               other.edgeConstraints.end());
    }

    bool overlap(const Constraints& other) const
    {
        for (const auto& vc : vertexConstraints)
        {
            if (other.vertexConstraints.count(vc) > 0)
            {
                return true;
            }
        }

        for (const auto& ec : edgeConstraints)
        {
            if (other.edgeConstraints.count(ec) > 0)
            {
                return true;
            }
        }

        return false;
    }

    friend ostream& operator<<(ostream& os, const Constraints& c)
    {
        for (const auto& vc : c.vertexConstraints)
        {
            os << vc << endl;
        }

        for (const auto& ec : c.edgeConstraints)
        {
            os << ec << endl;
        }

        return os;
    }
};


///
class Environment
{
private:
    int num_columns;
    int num_rows;
    unordered_set<Location> obstacles;
    vector<Location> m_goals;
    // vector< vector<int> > m_heuristic;
    size_t m_agentIdx;
    const Constraints* m_constraints;
    int m_lastGoalConstraint;
    int m_highLevelExpanded;
    int m_lowLevelExpanded;
    bool m_disappearAtGoal;

public:
    Environment(size_t dimx, size_t dimy, unordered_set<Location> obstacles,
                vector<Location> input_goals, bool disappearAtGoal = false)
            : num_columns(dimx),
              num_rows(dimy),
              obstacles(move(obstacles)),
              m_goals(move(input_goals)),
              m_agentIdx(0),
              m_constraints(nullptr),
              m_lastGoalConstraint(-1),
              m_highLevelExpanded(0),
              m_lowLevelExpanded(0),
              m_disappearAtGoal(disappearAtGoal)
    {}

    Environment(const Environment&) = delete;
    Environment& operator=(const Environment&) = delete;

    void setLowLevelContext(size_t agentIdx, const Constraints* constraints)
    {
        assert(constraints);  // NOLINT
        m_agentIdx = agentIdx;
        m_constraints = constraints;
        m_lastGoalConstraint = -1;
        for (const auto& vc : constraints->vertexConstraints)
        {
            if (vc.x == m_goals[m_agentIdx].x && vc.y == m_goals[m_agentIdx].y)
            {
                m_lastGoalConstraint = max(m_lastGoalConstraint, vc.time);
            }
        }
    }

    int admissible_heuristic(const TimeLocation& s)
    {
        // cout << "H: " <<  s << " " << m_heuristic[m_agentIdx][s.x + num_columns *
        // s.y] << endl;
        // return m_heuristic[m_agentIdx][s.x + num_columns * s.y];
        return abs(s.x - m_goals[m_agentIdx].x) +
               abs(s.y - m_goals[m_agentIdx].y);
    }

    bool is_solution(const TimeLocation& s)
    {
        return s.x == m_goals[m_agentIdx].x
               && s.y == m_goals[m_agentIdx].y
               && s.time > m_lastGoalConstraint;
    }

    void get_neighbors(const TimeLocation& time_location, vector<Neighbor<TimeLocation, Action, int> >& neighbors)
    {
        // cout << "#VC " << constraints.vertexConstraints.size() << endl;
        // for(const auto& vc : constraints.vertexConstraints) {
        //   cout << "  " << vc.time << "," << vc.x << "," << vc.y <<
        //   endl;
        // }
        neighbors.clear();

        TimeLocation wait_neighbor(time_location.time + 1, time_location.x, time_location.y);
        if (location_valid(wait_neighbor) && transition_valid(time_location, wait_neighbor))
        {
            neighbors.emplace_back(Neighbor<TimeLocation, Action, int>(wait_neighbor, Action::Wait, 1));
        }

        TimeLocation west_neighbor(time_location.time + 1, time_location.x - 1, time_location.y);
        if (location_valid(west_neighbor) && transition_valid(time_location, west_neighbor))
        {
            neighbors.emplace_back(Neighbor<TimeLocation, Action, int>(west_neighbor, Action::Left, 1));
        }

        TimeLocation east_neighbor(time_location.time + 1, time_location.x + 1, time_location.y);
        if (location_valid(east_neighbor) && transition_valid(time_location, east_neighbor))
        {
            neighbors.emplace_back(Neighbor<TimeLocation, Action, int>(east_neighbor, Action::Right, 1));
        }

        TimeLocation north_neighbor(time_location.time + 1, time_location.x, time_location.y + 1);
        if (location_valid(north_neighbor) && transition_valid(time_location, north_neighbor))
        {
            neighbors.emplace_back(Neighbor<TimeLocation, Action, int>(north_neighbor, Action::Up, 1));
        }

        TimeLocation south_neighbor(time_location.time + 1, time_location.x, time_location.y - 1);
        if (location_valid(south_neighbor) && transition_valid(time_location, south_neighbor))
        {
            neighbors.emplace_back(Neighbor<TimeLocation, Action, int>(south_neighbor, Action::Down, 1));
        }
    }

    bool getFirstConflict(const vector<PlanResult<TimeLocation, Action, int> >& solution, Conflict& result)
    {
        int max_t = 0;
        for (const auto& sol : solution)
        {
            max_t = max<int>(max_t, sol.path.size() - 1);
        }

        for (int t = 0; t <= max_t; ++t)
        {
            // check drive-drive vertex collisions
            for (size_t i = 0; i < solution.size(); ++i)
            {
                TimeLocation state1 = getState(i, solution, t);
                for (size_t j = i + 1; j < solution.size(); ++j)
                {
                    TimeLocation state2 = getState(j, solution, t);
                    if (state1.equalExceptTime(state2))
                    {
                        result.time = t;
                        result.agent1 = i;
                        result.agent2 = j;
                        result.type = Conflict::Vertex;
                        result.x1 = state1.x;
                        result.y1 = state1.y;
                        // cout << "VC " << t << "," << state1.x << "," << state1.y <<
                        // endl;

                        return true;
                    }
                }
            }

            // drive-drive edge (swap)
            for (size_t i = 0; i < solution.size(); ++i)
            {
                TimeLocation state1a = getState(i, solution, t);
                TimeLocation state1b = getState(i, solution, t + 1);

                for (size_t j = i + 1; j < solution.size(); ++j)
                {
                    TimeLocation state2a = getState(j, solution, t);
                    TimeLocation state2b = getState(j, solution, t + 1);
                    if (state1a.equalExceptTime(state2b) && state1b.equalExceptTime(state2a))
                    {
                        result.time = t;
                        result.agent1 = i;
                        result.agent2 = j;
                        result.type = Conflict::Edge;
                        result.x1 = state1a.x;
                        result.y1 = state1a.y;
                        result.x2 = state1b.x;
                        result.y2 = state1b.y;

                        return true;
                    }
                }
            }
        }

        return false;
    }

    void createConstraintsFromConflict(const Conflict& conflict, map<size_t, Constraints>& constraints)
    {
        if (conflict.type == Conflict::Vertex)
        {
            Constraints c1;
            c1.vertexConstraints.emplace(
                    VertexConstraint(conflict.time, conflict.x1, conflict.y1));
            constraints[conflict.agent1] = c1;
            constraints[conflict.agent2] = c1;
        }
        else if (conflict.type == Conflict::Edge)
        {
            Constraints c1;
            c1.edgeConstraints.emplace(EdgeConstraint(
                    conflict.time, conflict.x1, conflict.y1, conflict.x2, conflict.y2));
            constraints[conflict.agent1] = c1;
            Constraints c2;
            c2.edgeConstraints.emplace(EdgeConstraint(
                    conflict.time, conflict.x2, conflict.y2, conflict.x1, conflict.y1));
            constraints[conflict.agent2] = c2;
        }
    }

    void onExpandHighLevelNode(int /*cost*/)
    {
        m_highLevelExpanded++;
    }

    void onExpandLowLevelNode(const TimeLocation& /*s*/, int /*fScore*/, int /*gScore*/)
    {
        m_lowLevelExpanded++;
    }

    int highLevelExpanded()
    {
        return m_highLevelExpanded;
    }

    int lowLevelExpanded() const
    {
        return m_lowLevelExpanded;
    }

    TimeLocation getState(size_t agentIdx,
                          const vector<PlanResult<TimeLocation, Action, int> >& solution,
                          size_t t)
    {
        assert(agentIdx < solution.size());

        if (t < solution[agentIdx].path.size())
        {
            return solution[agentIdx].path[t].first;
        }

        assert(!solution[agentIdx].path.empty());

        if (m_disappearAtGoal)
        {
            // This is a trick to avoid changing the rest of the code significantly
            // After an agent disappeared, put it at a unique but invalid position
            // This will cause all calls to equalExceptTime(.) to return false.
            return TimeLocation(-1, -1 * (agentIdx + 1), -1);
        }

        return solution[agentIdx].path.back().first;
    }

    bool location_valid(const TimeLocation& s)
    {
        assert(m_constraints);
        const auto& con = m_constraints->vertexConstraints;

        return s.x >= 0 && s.x < num_columns && s.y >= 0 && s.y < num_rows &&
               obstacles.find(Location(s.x, s.y)) == obstacles.end() &&
               con.find(VertexConstraint(s.time, s.x, s.y)) == con.end();
    }

    bool transition_valid(const TimeLocation& s1, const TimeLocation& s2)
    {
        assert(m_constraints);
        const auto& con = m_constraints->edgeConstraints;

        return con.find(EdgeConstraint(s1.time, s1.x, s1.y, s2.x, s2.y)) == con.end();
    }
};

int main(int argc, char* argv[])
{
    namespace po = boost::program_options;
    // Declare the supported options.
    po::options_description desc("Allowed options");
    string inputFile;
    string outputFile;
    bool disappearAtGoal;
    desc.add_options()("help", "produce help message")
            ("input,i", po::value<string>(&inputFile)->required(), "input file (YAML)")
            ("output,o", po::value<string>(&outputFile)->required(), "output file (YAML)")
            ("disappear-at-goal", po::bool_switch(&disappearAtGoal), "make agents to disappear at goal rather than staying there");

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

    Environment mapf(dimx, dimy, obstacles, goals, disappearAtGoal);
    CBS<Action, Conflict, Constraints, Environment> cbs(mapf);
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