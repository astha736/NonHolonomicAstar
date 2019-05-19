cmake_minimum_required (VERSION 2.6)
project (NonHolonmicAstar)
add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE Release)

find_package(OpenCV REQUIRED)

find_package(PkgConfig)
pkg_check_modules(YamlCpp yaml-cpp)

include_directories(${YamlCpp_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} include)

add_library(NonHolonmicAstar SHARED 
	include/maze.h 
	include/a_star.h
    include/point.h 
    include/position.h 
    include/robotPose.h 
    src/point.cpp 
    src/position.cpp
    src/robotPose.cpp)

target_link_libraries(NonHolonmicAstar ${OpenCV_LIBS})

add_executable(cell src/solve_cell.cpp)
target_link_libraries(cell  NonHolonmicAstar)