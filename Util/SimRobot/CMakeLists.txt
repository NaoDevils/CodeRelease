cmake_minimum_required(VERSION 3.20)

project(Simulator)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

add_subdirectory(Src/SimRobot)
add_subdirectory(Src/SimRobotCore2)
add_subdirectory(Src/SimRobotEditor)

add_dependencies(SimRobot SimRobotCore2 SimRobotEditor)
