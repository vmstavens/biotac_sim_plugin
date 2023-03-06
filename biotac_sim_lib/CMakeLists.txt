cmake_minimum_required(VERSION 3.4)
project(biotac_sim_lib)

set(ROS_BUILD_TYPE Release)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    eigen_conversions
    sr_robot_msgs
    ros_utils_cpp
    gazebo_ros
)

find_package(gazebo REQUIRED)

find_package(Eigen3 REQUIRED)

include_directories(include
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

link_directories(
    ${GAZEBO_LIBRARY_DIRS}
)

find_package(OpenMP)
if (OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

catkin_package(
    INCLUDE_DIRS
    LIBRARIES
        biotac_sim_lib
    CATKIN_DEPENDS
        roscpp
        eigen_conversions
        sr_robot_msgs
        ros_utils_cpp
    DEPENDS
        gazebo_ros
        SDF
        EIGEN3
    INCLUDE_DIRS
        include
)

add_definitions(-fPIC)

add_library(biotac_sim_lib src/biotac_sim_lib.cpp)
add_dependencies(biotac_sim_lib ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(biotac_sim_lib ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} yaml-cpp)

install(TARGETS
    biotac_sim_lib
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY 
    include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
