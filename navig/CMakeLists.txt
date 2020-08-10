cmake_minimum_required(VERSION 2.8.3)
project(navig)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE DEBUG)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED actionlib costmap_2d COMPONENTS
  geometry_msgs
  dynamic_reconfigure
  nav_msgs
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  tf2
  cv_bridge
  image_transport
  actionlib
  actionlib_msgs
  ar_track_alvar_msgs
  multi_level_map_messages
)
find_package(PCL 1.8 REQUIRED)
#set(PCL_INCLUDE_DIRS /usr/local/include/pcl-1.9 /usr/include/eigen3/)

## System dependencies are found with CMake's conventions
#find_package(Boost REQUIRED COMPONENTS system)

#add_message_files(
 # FILES
  #PoseOfCluster.msg
#)

generate_messages(
   DEPENDENCIES
   std_msgs
   actionlib_msgs
   nav_msgs
   multi_level_map_messages
 )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES navig
  CATKIN_DEPENDS geometry_msgs nav_msgs pcl_conversions pcl_ros roscpp rospy sensor_msgs std_msgs tf actionlib actionlib_msgs
  DEPENDS system_lib PCL
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

## Declare a C++ library
 ##add_library(${PROJECT_NAME}
   #src/grid_layer.cpp
 #)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/navig_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

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

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_navig.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

add_executable(normal_estimation_using_integral_images src/normal_estimation_using_integral_images.cpp)
target_link_libraries(normal_estimation_using_integral_images ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_executable(moment_of_inertia src/moment_of_inertia.cpp)
target_link_libraries(moment_of_inertia ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_executable(clustering src/clustering.cpp)
target_link_libraries(clustering ${catkin_LIBRARIES} ${PCL_LIBRARIES})
#add_executable(don src/don.cpp)
#target_link_libraries(don ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_executable(stair_traversal src/stairtraversal.cpp)
target_link_libraries(stair_traversal ${catkin_LIBRARIES})
add_executable(frontiers src/frontier.cpp)
target_link_libraries(frontiers ${catkin_LIBRARIES})
add_executable(seg_clustering src/seg_clustering.cpp)
target_link_libraries(seg_clustering ${catkin_LIBRARIES})
