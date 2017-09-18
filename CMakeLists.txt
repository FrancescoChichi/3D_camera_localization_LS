cmake_minimum_required(VERSION 3.5)
project(Probabilistic_robotics)

set(CMAKE_CXX_STANDARD 11)
include_directories(/home/francesco/eigen)
set(SOURCE_FILES
        main.cpp
        Landmark.h
        Pose.h
        utils.cpp utils.h
        distance_map.cpp distance_map.h
        points_utils.h points_utils.cpp
        Transition.h
        defs.h
        p3p_solver.cpp
        p3p_solver.h
        distance_map_correspondence_finder.cpp
        distance_map_correspondence_finder.h
        camera.h camera.cpp
        Observation.h g2o_parser.h ProjectPoint.h DictPoints.h Test.h)
add_executable(Probabilistic_robotics ${SOURCE_FILES})



# Find Opencv
FIND_PACKAGE(OpenCV  REQUIRED)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
target_link_libraries( Probabilistic_robotics ${OpenCV_LIBS} )