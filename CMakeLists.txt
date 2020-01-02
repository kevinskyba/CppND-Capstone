cmake_minimum_required(VERSION 3.15)
project(CppND_Capstone)

set(CMAKE_CXX_STANDARD 17)

###
# Files
###

file(GLOB_RECURSE sources src/*.cpp src/*.h)

###
# Targets
###

add_executable(CppND_Capstone ${sources})

###
# Dependencies
###

FIND_PACKAGE(freenect2 REQUIRED)
include_directories(
        ${freenect2_INCLUDE_DIRS}
)
target_link_libraries (
        CppND_Capstone
        ${freenect2_LIBRARIES}
)

FIND_PACKAGE(PCL 1.3 REQUIRED)
include_directories(
        ${PCL_INCLUDE_DIRS}
)
link_directories(
        ${PCL_LIBRARY_DIRS}
)
add_definitions(
        ${PCL_DEFINITIONS}
)
target_link_libraries (
        CppND_Capstone
        ${PCL_LIBRARIES}
)