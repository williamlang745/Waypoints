cmake_minimum_required(VERSION 3.9)

project(PathFinder VERSION 1.0.0 DESCRIPTION "Library to find the shortest path through a 2D map with obstacles.")

add_library(PathFinder STATIC src/PathFinder.cpp)

set_target_properties(PathFinder PROPERTIES PUBLIC_HEADER inc/PathFinder.h)

target_include_directories(PathFinder PUBLIC inc)
