cmake_minimum_required(VERSION 3.9)

project(App DESCRIPTION "Example application showing how to interface with the PathFinderLib.")

add_executable(App main.cpp)

target_link_libraries(App PRIVATE PathFinder)