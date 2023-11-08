//
// Created by take_ on 2023/11/8.
//

#ifndef LIBMULTIROBOTPLANNING_LOW_LEVEL_HPP
#define LIBMULTIROBOTPLANNING_LOW_LEVEL_HPP

#include <boost/heap/fibonacci_heap.hpp>
#include <boost/heap/d_ary_heap.hpp>
#include <fstream>
#include <iostream>
#include <unordered_map>
#include <unordered_set>

#include "neighbor.hpp"
#include "planresult.hpp"
#include "util.hpp"

using libMultiRobotPlanning::Neighbor;
using libMultiRobotPlanning::PlanResult;

/*!
  \example a_star.cpp Simple example using a 2D grid world and
  up/down/left/right
  actions
*/

/*! \brief A* Algorithm to find the shortest path

This class implements the A* algorithm. A* is an informed search algorithm
that finds the shortest path for a given map. It can use a heuristic that
needsto be admissible.

This class can either use a fibonacci heap, or a d-ary heap. The latter is the
default. Define "USE_FIBONACCI_HEAP" to use the fibonacci heap instead.

\tparam Location Custom location for the search. Needs to be copy'able
\tparam Action Custom action for the search. Needs to be copy'able
\tparam Cost Custom Cost type (integer or floating point types)
\tparam Environment This class needs to provide the custom A* logic. In
    particular, it needs to support the following functions:
  - `Cost admissible_heuristic(const Location& s)`\n
    This function can return 0 if no suitable heuristic is available.

  - `bool is_solution(const Location& s)`\n
    Return true if the given location is a goal location.

  - `void get_neighbors(const Location& s, std::vector<Neighbor<Location, Action,
   int> >& neighbors)`\n
    Fill the list of neighboring location for the given location s.

  - `void onExpandNode(const Location& s, int f_score, int g_score)`\n
    This function is called on every expansion and can be used for statistical
purposes.

  - `void onDiscover(const Location& s, int f_score, int g_score)`\n
    This function is called on every node discovery and can be used for
   statistical purposes.

    \tparam LocationHasher A class to convert a location to a hash value. Default:
   std::hash<Location>
*/



#endif //LIBMULTIROBOTPLANNING_LOW_LEVEL_HPP
