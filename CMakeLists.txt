cmake_minimum_required(VERSION 3.5)
project(libMultiRobotPlanning)

set(CMAKE_TOOLCHAIN_FILE "C:/vcpkg/scripts/buildsystems/vcpkg.cmake")
set(CMAKE_PREFIX_PATH "C:/vcpkg/installed/x64-windows")

find_package(Boost REQUIRED COMPONENTS program_options)

include_directories(${Boost_INCLUDE_DIR})

find_package(yaml-cpp CONFIG REQUIRED)


# clang-tidy target (linter & static code analysis)
add_custom_target(clang-tidy
        COMMAND CMAKE_EXPORT_COMPILE_COMMANDS=ON run-clang-tidy ${CMAKE_CURRENT_SOURCE_DIR})

# tests
add_custom_target(run-test
        COMMAND python3 -m unittest discover -s ${CMAKE_CURRENT_SOURCE_DIR}/test
)

add_custom_target(everything
        DEPENDS clang-format clang-tidy docs run-test
)

# Enable C++14 and warnings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


# Creates compile database used by clang-tidy.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include_directories(include)

# Examples

## a_star
add_executable(a_star example/a_star.cpp)
target_link_libraries(a_star ${Boost_LIBRARIES})

## a_star_isolated
add_executable(a_star_isolated
        a_star_isolated.hpp a_star_isolated.cpp
        include/libMultiRobotPlanning/util.hpp)
target_link_libraries(a_star_isolated ${Boost_LIBRARIES})

## a_star_epsilon
add_executable(a_star_epsilon example/a_star_epsilon.cpp)
target_link_libraries(a_star_epsilon ${Boost_LIBRARIES})

## a_star_epsilon_isolated
add_executable(a_star_epsilon_isolated example/a_star_epsilon_isolated.cpp)
target_link_libraries(a_star_epsilon_isolated ${Boost_LIBRARIES})

## assignment
add_executable(assignment example/assignment.cpp)
target_link_libraries(assignment ${Boost_LIBRARIES})

## cbs_roadmap
add_executable(cbs_roadmap example/cbs_roadmap.cpp)
target_link_libraries(cbs_roadmap ${Boost_LIBRARIES} yaml-cpp::yaml-cpp)

## cbs
add_executable(cbs example/cbs.cpp)
target_link_libraries(cbs ${Boost_LIBRARIES} yaml-cpp::yaml-cpp)

## cbs_isolated
add_executable(cbs_isolated
        include/libMultiRobotPlanning/cbs_isolated.hpp example/cbs_isolated.cpp
        include/libMultiRobotPlanning/util.hpp)
target_link_libraries(cbs_isolated ${Boost_LIBRARIES} yaml-cpp::yaml-cpp)

## cbs_ta
add_executable(cbs_ta example/cbs_ta.cpp)
target_link_libraries(cbs_ta ${Boost_LIBRARIES} yaml-cpp::yaml-cpp)

## cbs_ta_isolated
add_executable(cbs_ta_isolated
        include/libMultiRobotPlanning/cbs_ta_isolated.hpp example/cbs_ta_isolated.cpp)
target_link_libraries(cbs_ta_isolated ${Boost_LIBRARIES} yaml-cpp::yaml-cpp)

## ecbs
add_executable(ecbs example/ecbs.cpp)
target_link_libraries(ecbs ${Boost_LIBRARIES} yaml-cpp::yaml-cpp)

## ecbs_isolated
add_executable(ecbs_isolated example/ecbs_isolated.cpp)
target_link_libraries(ecbs_isolated ${Boost_LIBRARIES} yaml-cpp::yaml-cpp)

## ecbs_ta
add_executable(ecbs_ta example/ecbs_ta.cpp)
target_link_libraries(ecbs_ta ${Boost_LIBRARIES} yaml-cpp::yaml-cpp)

## next_best_assignment
add_executable(next_best_assignment example/next_best_assignment.cpp)
target_link_libraries(next_best_assignment ${Boost_LIBRARIES} yaml-cpp::yaml-cpp)

## sipp
add_executable(sipp example/sipp.cpp)
target_link_libraries(sipp ${Boost_LIBRARIES} yaml-cpp::yaml-cpp)

## sipp_isolated
add_executable(sipp_isolated example/sipp_isolated.cpp)
target_link_libraries(sipp_isolated ${Boost_LIBRARIES} yaml-cpp::yaml-cpp)

## mapf_prioritized_sipp
add_executable(mapf_prioritized_sipp example/mapf_prioritized_sipp.cpp)
target_link_libraries(mapf_prioritized_sipp ${Boost_LIBRARIES} yaml-cpp::yaml-cpp)
