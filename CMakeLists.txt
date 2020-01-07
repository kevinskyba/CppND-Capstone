cmake_minimum_required(VERSION 3.15)
project(CppND_Capstone)

set(CMAKE_CXX_STANDARD 17)

###
# Args
###

option(KINECT "Kinect" FALSE)

###
# Files
###

file(COPY data/cubes/ DESTINATION ./)

if (KINECT)
    set(kinect_viewer_sources

            src/KinectViewer.cpp
            src/KinectDepthCapture.cpp
            src/DepthFrame.cpp

            src/cxxopts.h
            src/FrameUtils.h
            src/KinectDepthCapture.h
            src/DepthFrame.h)

    set(capture_sources

            src/Capture.cpp
            src/KinectDepthCapture.cpp
            src/DepthFrame.cpp

            src/cxxopts.h
            src/FrameUtils.h
            src/KinectDepthCapture.h
            src/DepthFrame.h)
endif()

set(file_viewer_sources

        src/FileViewer.cpp
        src/FileDepthCapture.cpp
        src/DepthFrame.cpp

        src/cxxopts.h
        src/FrameUtils.h
        src/FileDepthCapture.h
        src/DepthFrame.h)


set(align_sources

        src/Align.cpp
        src/FileDepthCapture.cpp
        src/DepthFrame.cpp

        src/cxxopts.h
        src/FrameUtils.h
        src/FileDepthCapture.h
        src/DepthFrame.h)

###
# Targets
###

if (KINECT)
    add_executable(CppND_Capstone_Kinect_Viewer ${kinect_viewer_sources})
    add_executable(CppND_Capstone_Capture ${capture_sources})
endif()
add_executable(CppND_Capstone_File_Viewer ${file_viewer_sources})
add_executable(CppND_Capstone_Align ${align_sources})

###
# Dependencies
###

# Freenect
if (KINECT)
    FIND_PACKAGE(freenect2 REQUIRED)
    include_directories(
            ${freenect2_INCLUDE_DIRS}
    )
    target_link_libraries (
            CppND_Capstone_Kinect_Viewer
            ${freenect2_LIBRARIES}
    )
    target_link_libraries (
            CppND_Capstone_Capture
            ${freenect2_LIBRARIES}
    )
endif()

# PCL
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
if (KINECT)
    target_link_libraries (
            CppND_Capstone_Kinect_Viewer
            ${PCL_LIBRARIES}
    )
    target_link_libraries (
            CppND_Capstone_Capture
            ${PCL_LIBRARIES}
    )
endif()
target_link_libraries (
        CppND_Capstone_File_Viewer
        ${PCL_LIBRARIES}
)

target_link_libraries (
        CppND_Capstone_Align
        ${PCL_LIBRARIES}
)

# Super4PCS
FIND_PACKAGE(Super4PCS REQUIRED)
include_directories(
        ${Super4PCS_INCLUDE_DIRS}
)
link_directories(
        ${Super4PCS_LIBRARY_DIRS}
)
add_definitions(
        ${Super4PCS_DEFINITIONS}
)
if (KINECT)
    target_link_libraries (
            CppND_Capstone_Kinect_Viewer
            ${Super4PCS_LIBRARIES}
    )
    target_link_libraries (
            CppND_Capstone_Capture
            ${Super4PCS_LIBRARIES}
    )
endif()
target_link_libraries (
        CppND_Capstone_File_Viewer
        ${Super4PCS_LIBRARIES}
)

target_link_libraries (
        CppND_Capstone_Align
        ${Super4PCS_LIBRARIES}
)

# stdc++fs
if (KINECT)
    target_link_libraries (
            CppND_Capstone_Kinect_Viewer
            stdc++fs
    )
    target_link_libraries (
            CppND_Capstone_Capture
            stdc++fs
    )
endif()
target_link_libraries (
        CppND_Capstone_File_Viewer
        stdc++fs
)
target_link_libraries (
        CppND_Capstone_Align
        stdc++fs
)