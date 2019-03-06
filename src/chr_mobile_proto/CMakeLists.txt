cmake_minimum_required(VERSION 2.8.3)
project(chr_mobile_proto)

set(CMAKE_CXX_FLAGS "-MMD -MP -Wall -Wextra -O0 -std=c++11")

find_package(catkin REQUIRED roscpp std_msgs)

# generate_messages(
# 	DEPENDENCIES
# 	std_msgs  # Or other packages containing msgs
# )
catkin_package(
	INCLUDE_DIRS include
#	LIBRARIES chr_mobile_proto
#	CATKIN_DEPENDS other_catkin_pkg
#	DEPENDS system_lib
)

include_directories(
	include ${catkin_INCLUDE_DIRS}
)
add_library(
	prop STATIC src/mobile_propulsion.cpp src/common_functions.cpp src/arduino.cpp
)

add_library(
	steer STATIC src/mobile_steering.cpp src/epos2.cpp src/common_functions.cpp
)

# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(propulsion_node src/propulsion_node.cpp)

add_executable(steering_node src/steering_node.cpp)

add_executable(rasp_node src/rasp_node.cpp)

target_link_libraries(propulsion_node	${catkin_LIBRARIES} prop)
target_link_libraries(steering_node	${catkin_LIBRARIES} steer EposCmd)
target_link_libraries(rasp_node	${catkin_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_chr_mobile_proto.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
