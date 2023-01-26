cmake_minimum_required(VERSION 3.4)
project(biotac_sim_demo)

# find catkin and any catkin packages
find_package(
	catkin REQUIRED
	COMPONENTS
		rospy
)

# declare a catkin package
catkin_package(
	INCLUDE_DIRS
	LIBRARIES
	CATKIN_DEPENDS
		rospy
	DEPENDS
)

## process setup.py
catkin_python_setup()

## install python nodes
catkin_install_python(
  PROGRAMS 
    examples/biotac_sim_demo.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

