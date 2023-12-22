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



///
enum class Action {
    Up,
    Down,
    Left,
    Right,
    Wait,
};

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

struct Conflict {
    enum Type {
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

    friend std::ostream& operator<<(std::ostream& os, const Conflict& c) {
        switch (c.type) {
            case Vertex:
                return os << c.time << ": Vertex(" << c.x1 << "," << c.y1 << ")";
            case Edge:
                return os << c.time << ": Edge(" << c.x1 << "," << c.y1 << "," << c.x2
                          << "," << c.y2 << ")";
        }
        return os;
    }
};

struct VertexConstraint {
    VertexConstraint(int time, int x, int y) : time(time), x(x), y(y) {}
    int time;
    int x;
    int y;

    bool operator<(const VertexConstraint& other) const {
        return std::tie(time, x, y) < std::tie(other.time, other.x, other.y);
    }

    bool operator==(const VertexConstraint& other) const {
        return std::tie(time, x, y) == std::tie(other.time, other.x, other.y);
    }

    friend std::ostream& operator<<(std::ostream& os, const VertexConstraint& c) {
        return os << "VC(" << c.time << "," << c.x << "," << c.y << ")";
    }
};

namespace std {
    template <>
    struct hash<VertexConstraint> {
        size_t operator()(const VertexConstraint& s) const {
            size_t seed = 0;
            boost::hash_combine(seed, s.time);
            boost::hash_combine(seed, s.x);
            boost::hash_combine(seed, s.y);
            return seed;
        }
    };
}  // namespace std

struct EdgeConstraint {
    EdgeConstraint(int time, int x1, int y1, int x2, int y2)
            : time(time), x1(x1), y1(y1), x2(x2), y2(y2) {}
    int time;
    int x1;
    int y1;
    int x2;
    int y2;

    bool operator<(const EdgeConstraint& other) const {
        return std::tie(time, x1, y1, x2, y2) <
               std::tie(other.time, other.x1, other.y1, other.x2, other.y2);
    }

    bool operator==(const EdgeConstraint& other) const {
        return std::tie(time, x1, y1, x2, y2) ==
               std::tie(other.time, other.x1, other.y1, other.x2, other.y2);
    }

    friend std::ostream& operator<<(std::ostream& os, const EdgeConstraint& c) {
        return os << "EC(" << c.time << "," << c.x1 << "," << c.y1 << "," << c.x2
                  << "," << c.y2 << ")";
    }
};

namespace std {
    template <>
    struct hash<EdgeConstraint> {
        size_t operator()(const EdgeConstraint& s) const {
            size_t seed = 0;
            boost::hash_combine(seed, s.time);
            boost::hash_combine(seed, s.x1);
            boost::hash_combine(seed, s.y1);
            boost::hash_combine(seed, s.x2);
            boost::hash_combine(seed, s.y2);
            return seed;
        }
    };
}  // namespace std

struct Constraints {
    std::unordered_set<VertexConstraint> vertexConstraints;
    std::unordered_set<EdgeConstraint> edgeConstraints;

    void add(const Constraints& other) {
        vertexConstraints.insert(other.vertexConstraints.begin(),
                                 other.vertexConstraints.end());
        edgeConstraints.insert(other.edgeConstraints.begin(),
                               other.edgeConstraints.end());
    }

    bool overlap(const Constraints& other) const {
        for (const auto& vc : vertexConstraints) {
            if (other.vertexConstraints.count(vc) > 0) {
                return true;
            }
        }
        for (const auto& ec : edgeConstraints) {
            if (other.edgeConstraints.count(ec) > 0) {
                return true;
            }
        }
        return false;
    }

    friend std::ostream& operator<<(std::ostream& os, const Constraints& c) {
        for (const auto& vc : c.vertexConstraints) {
            os << vc << std::endl;
        }
        for (const auto& ec : c.edgeConstraints) {
            os << ec << std::endl;
        }
        return os;
    }
};


///
class Environment {
public:
    Environment(size_t dimx, size_t dimy, std::unordered_set<Location> obstacles,
                std::vector<Location> goals, bool disappearAtGoal = false)
            : num_columns(dimx),
              num_rows(dimy),
              obstacles(std::move(obstacles)),
              m_goals(std::move(goals)),
              m_agentIdx(0),
              m_constraints(nullptr),
              m_lastGoalConstraint(-1),
              m_highLevelExpanded(0),
              m_lowLevelExpanded(0),
              m_disappearAtGoal(disappearAtGoal)
    {
    }

    Environment(const Environment&) = delete;
    Environment& operator=(const Environment&) = delete;

    void setLowLevelContext(size_t agentIdx, const Constraints* constraints) {
        assert(constraints);  // NOLINT
        m_agentIdx = agentIdx;
        m_constraints = constraints;
        m_lastGoalConstraint = -1;
        for (const auto& vc : constraints->vertexConstraints) {
            if (vc.x == m_goals[m_agentIdx].x && vc.y == m_goals[m_agentIdx].y) {
                m_lastGoalConstraint = std::max(m_lastGoalConstraint, vc.time);
            }
        }
    }

    int admissible_heuristic(const TimeLocation& s) {
        return std::abs(s.location.x - m_goals[m_agentIdx].x) +
               std::abs(s.location.y - m_goals[m_agentIdx].y);
    }

    // low-level
    int focalStateHeuristic(
            const TimeLocation& s, int /*gScore*/,
            const std::vector<PlanResult<TimeLocation, Action, int> >& solution) {
        int numConflicts = 0;
        for (size_t i = 0; i < solution.size(); ++i) {
            if (i != m_agentIdx && !solution[i].path.empty()) {
                TimeLocation state2 = getState(i, solution, s.time_step);
                if (s.location == state2.location) {
                    ++numConflicts;
                }
            }
        }
        return numConflicts;
    }

    // low-level
    int focalTransitionHeuristic(
            const TimeLocation& s1a, const TimeLocation& s1b, int /*gScoreS1a*/, int /*gScoreS1b*/,
            const std::vector<PlanResult<TimeLocation, Action, int> >& solution) {
        int numConflicts = 0;
        for (size_t i = 0; i < solution.size(); ++i) {
            if (i != m_agentIdx && !solution[i].path.empty()) {
                TimeLocation s2a = getState(i, solution, s1a.time_step);
                TimeLocation s2b = getState(i, solution, s1b.time_step);
                if ((s1a.location==s2b.location) && (s1b.location == s2a.location))
                {
                    ++numConflicts;
                }
            }
        }
        return numConflicts;
    }

    // Count all conflicts
    int focalHeuristic(
            const std::vector<PlanResult<TimeLocation, Action, int> >& solution) {
        int numConflicts = 0;

        int max_t = 0;
        for (const auto& sol : solution) {
            max_t = std::max<int>(max_t, sol.path.size() - 1);
        }

        for (int t = 0; t < max_t; ++t) {
            // check drive-drive vertex collisions
            for (size_t i = 0; i < solution.size(); ++i) {
                TimeLocation state1 = getState(i, solution, t);
                for (size_t j = i + 1; j < solution.size(); ++j) {
                    TimeLocation state2 = getState(j, solution, t);
                    if (state1.location == state2.location) {
                        ++numConflicts;
                    }
                }
            }
            // drive-drive edge (swap)
            for (size_t i = 0; i < solution.size(); ++i) {
                TimeLocation state1a = getState(i, solution, t);
                TimeLocation state1b = getState(i, solution, t + 1);
                for (size_t j = i + 1; j < solution.size(); ++j) {
                    TimeLocation state2a = getState(j, solution, t);
                    TimeLocation state2b = getState(j, solution, t + 1);
                    if (state1a.location == state2b.location &&
                        state1b.location == state2a.location) {
                        ++numConflicts;
                    }
                }
            }
        }
        return numConflicts;
    }

    bool is_solution(const TimeLocation& s)
    {
        return s.location.x == m_goals[m_agentIdx].x && s.location.y == m_goals[m_agentIdx].y &&
               s.time_step > m_lastGoalConstraint;
    }

    void get_neighbors(const TimeLocation& s,
                       std::vector<Neighbor<TimeLocation, Action, int> >& neighbors) {
        // std::cout << "#VC " << constraints.vertexConstraints.size() << std::endl;
        // for(const auto& vc : constraints.vertexConstraints) {
        //   std::cout << "  " << vc.time << "," << vc.x << "," << vc.y <<
        //   std::endl;
        // }
        neighbors.clear();
        {
            TimeLocation n(s.time_step + 1, Location(s.location.x, s.location.y));
            if (location_valid(n) && transitionValid(s, n)) {
                neighbors.emplace_back(
                        Neighbor<TimeLocation, Action, int>(n, Action::Wait, 1));
            }
        }
        {
            TimeLocation n(s.time_step + 1, Location(s.location.x - 1, s.location.y));
            if (location_valid(n) && transitionValid(s, n)) {
                neighbors.emplace_back(
                        Neighbor<TimeLocation, Action, int>(n, Action::Left, 1));
            }
        }
        {
            TimeLocation n(s.time_step + 1, Location(s.location.x + 1, s.location.y));
            if (location_valid(n) && transitionValid(s, n)) {
                neighbors.emplace_back(
                        Neighbor<TimeLocation, Action, int>(n, Action::Right, 1));
            }
        }
        {
            TimeLocation n(s.time_step + 1, Location(s.location.x, s.location.y + 1));
            if (location_valid(n) && transitionValid(s, n)) {
                neighbors.emplace_back(Neighbor<TimeLocation, Action, int>(n, Action::Up, 1));
            }
        }
        {
            TimeLocation n(s.time_step + 1, Location(s.location.x, s.location.y - 1));
            if (location_valid(n) && transitionValid(s, n)) {
                neighbors.emplace_back(
                        Neighbor<TimeLocation, Action, int>(n, Action::Down, 1));
            }
        }
    }

    bool getFirstConflict(
            const std::vector<PlanResult<TimeLocation, Action, int> >& solution,
    Conflict& result) {
        int max_t = 0;
        for (const auto& sol : solution) {
            max_t = std::max<int>(max_t, sol.path.size() - 1);
        }

        for (int t = 0; t <= max_t; ++t) {
            // check drive-drive vertex collisions
            for (size_t i = 0; i < solution.size(); ++i) {
                TimeLocation state1 = getState(i, solution, t);
                for (size_t j = i + 1; j < solution.size(); ++j) {
                    TimeLocation state2 = getState(j, solution, t);
                    if (state1.location == state2.location) {
                        result.time = t;
                        result.agent1 = i;
                        result.agent2 = j;
                        result.type = Conflict::Vertex;
                        result.x1 = state1.location.x;
                        result.y1 = state1.location.y;
                        // std::cout << "VC " << t << "," << state1.x << "," << state1.y <<
                        // std::endl;
                        return true;
                    }
                }
            }
            // drive-drive edge (swap)
            for (size_t i = 0; i < solution.size(); ++i) {
                TimeLocation state1a = getState(i, solution, t);
                TimeLocation state1b = getState(i, solution, t + 1);
                for (size_t j = i + 1; j < solution.size(); ++j) {
                    TimeLocation state2a = getState(j, solution, t);
                    TimeLocation state2b = getState(j, solution, t + 1);
                    if (state1a.location == state2b.location &&
                        state1b.location == state2a.location) {
                        result.time = t;
                        result.agent1 = i;
                        result.agent2 = j;
                        result.type = Conflict::Edge;
                        result.x1 = state1a.location.x;
                        result.y1 = state1a.location.y;
                        result.x2 = state1b.location.x;
                        result.y2 = state1b.location.y;
                        return true;
                    }
                }
            }
        }

        return false;
    }

    void createConstraintsFromConflict(
            const Conflict& conflict, std::map<size_t, Constraints>& constraints) {
        if (conflict.type == Conflict::Vertex) {
            Constraints c1;
            c1.vertexConstraints.emplace(
                    VertexConstraint(conflict.time, conflict.x1, conflict.y1));
            constraints[conflict.agent1] = c1;
            constraints[conflict.agent2] = c1;
        } else if (conflict.type == Conflict::Edge) {
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

    void onExpandHighLevelNode(int /*cost*/) { m_highLevelExpanded++; }

    void onExpandLowLevelNode(const TimeLocation& /*s*/, int /*fScore*/,
                              int /*gScore*/) {
        m_lowLevelExpanded++;
    }

    int highLevelExpanded() { return m_highLevelExpanded; }

    int lowLevelExpanded() const { return m_lowLevelExpanded; }

private:
    TimeLocation getState(size_t agentIdx,
                   const std::vector<PlanResult<TimeLocation, Action, int> >& solution,
    size_t t) {
        assert(agentIdx < solution.size());
        if (t < solution[agentIdx].path.size()) {
            return solution[agentIdx].path[t].first;
        }
        assert(!solution[agentIdx].path.empty());
        if (m_disappearAtGoal) {
            // This is a trick to avoid changing the rest of the code significantly
            // After an agent disappeared, put it at a unique but invalid position
            // This will cause all calls to equalExceptTime(.) to return false.
            return TimeLocation(-1, Location(-1 * (agentIdx+1), -1));
        }
        return solution[agentIdx].path.back().first;
    }

    bool location_valid(const TimeLocation& s) {
        assert(m_constraints);
        const auto& con = m_constraints->vertexConstraints;
        return s.location.x >= 0 && s.location.x < num_columns
            && s.location.y >= 0 && s.location.y < num_rows &&
               obstacles.find(Location(s.location.x, s.location.y)) == obstacles.end() &&
               con.find(VertexConstraint(s.time_step, s.location.x, s.location.y)) == con.end();
    }

    bool transitionValid(const TimeLocation& s1, const TimeLocation& s2) {
        assert(m_constraints);
        const auto& con = m_constraints->edgeConstraints;
        return con.find(EdgeConstraint(s1.time_step, s1.location.x, s1.location.y, s2.location.x, s2.location.y)) ==
               con.end();
    }

private:
    int num_columns;
    int num_rows;
    std::unordered_set<Location> obstacles;
    std::vector<Location> m_goals;
    size_t m_agentIdx;
    const Constraints* m_constraints;
    int m_lastGoalConstraint;
    int m_highLevelExpanded;
    int m_lowLevelExpanded;
    bool m_disappearAtGoal;
};

int main(int argc, char* argv[]) {
    namespace po = boost::program_options;
    // Declare the supported options.
    po::options_description desc("Allowed options");
    std::string inputFile;
    std::string outputFile;
    bool disappearAtGoal;
    float w;
    desc.add_options()("help", "produce help message")(
            "input,i", po::value<std::string>(&inputFile)->required(),
            "input file (YAML)")("output,o",
                                 po::value<std::string>(&outputFile)->required(),
                                 "output file (YAML)")(
            "suboptimality,w", po::value<float>(&w)->default_value(1.0),
            "suboptimality bound")(
            "disappear-at-goal", po::bool_switch(&disappearAtGoal), "make agents to disappear at goal rather than staying there");

    try {
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
    std::unordered_set<TimeLocation> startStatesSet;
    for (const auto& s : startStates) {
        if (startStatesSet.find(s) != startStatesSet.end()) {
            std::cout << "Identical start locations detected -> no solution!" << std::endl;
            return 0;
        }
        startStatesSet.insert(s);
    }

    Environment mapf(dimx, dimy, obstacles, goals, disappearAtGoal);
    ECBS<TimeLocation, Action, int, Conflict, Constraints, Environment> ecbs(mapf, w);
    std::vector<PlanResult<TimeLocation, Action, int> > solution;

    Timer timer;
    bool success = ecbs.search(startStates, solution);
    timer.stop();

    if (success) {
        std::cout << "Planning successful! " << std::endl;
        int cost = 0;
        int makespan = 0;
        for (const auto& s : solution) {
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
        for (size_t a = 0; a < solution.size(); ++a) {
            // std::cout << "Solution for: " << a << std::endl;
            // for (size_t i = 0; i < solution[a].actions.size(); ++i) {
            //   std::cout << solution[a].path[i].second << ": " <<
            //   solution[a].path[i].first << "->" << solution[a].actions[i].first
            //   << "(cost: " << solution[a].actions[i].second << ")" << std::endl;
            // }
            // std::cout << solution[a].path.back().second << ": " <<
            // solution[a].path.back().first << std::endl;

            out << "  agent" << a << ":" << std::endl;
            for (const auto& state : solution[a].path) {
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
    } else {
        std::cout << "Planning NOT successful!" << std::endl;
    }

    return 0;
}