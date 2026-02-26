cmake_minimum_required(VERSION 3.14)
project(planning_controlling_sim_demo LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Python3 COMPONENTS Development REQUIRED)
find_package(Eigen3 QUIET)

add_executable(sim_demo
  main.cpp
  pid_controller.cc
  trajectory.cc
  lqr_controller.cc
  visualizer.cc
)

target_include_directories(sim_demo PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${Python3_INCLUDE_DIRS}
)

if(Eigen3_FOUND)
  target_link_libraries(sim_demo PRIVATE Eigen3::Eigen)
else()
  target_include_directories(sim_demo PRIVATE /usr/include/eigen3)
endif()

target_link_libraries(sim_demo PRIVATE ${Python3_LIBRARIES})
