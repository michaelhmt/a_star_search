cmake_minimum_required(VERSION 3.22)
project(c___astar)

set(CMAKE_CXX_STANDARD 23)
set(PYTHON_EXECUTABLE "C:/Users/Michael/anaconda3/envs/a_star_search/python.exe")

add_subdirectory(extern/pybind11)
add_executable(c___astar main.cpp astar_search.cpp astar_search.h)
target_link_libraries(c___astar PRIVATE pybind11::pybind11)
