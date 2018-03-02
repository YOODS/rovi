cmake_minimum_required(VERSION 2.8.3)
project(rovi)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  cv_bridge
  message_generation
)

add_service_files(
  FILES
  dialog.srv
  DigitalFilter.srv
  ImageFilter.srv
  GenPC.srv
  GetGrid.srv
  GetCalibs.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rovi
#  CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs cv_bridge message_runtime
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs cv_bridge
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/rovi.cpp
#	src/libs/CircleCalibBoard.cpp
#)

#add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#add_executable(${PROJECT_NAME}_node src/rovi_node.cpp)
#add_executable(grid_node src/grid_node.cpp src/libs/CircleCalibBoard.cpp)

add_executable(main_node src/main_node.cpp)
target_link_libraries(main_node ${catkin_LIBRARIES})
add_dependencies(main_node rovi_gencpp)

add_executable(remap_node src/remap_node.cpp)
target_link_libraries(remap_node ${catkin_LIBRARIES})
add_dependencies(remap_node rovi_gencpp)

add_executable(genpc_node src/genpc_node.cpp)
target_link_libraries(genpc_node ${catkin_LIBRARIES})
add_dependencies(genpc_node rovi_gencpp)

add_executable(tools_node src/tools_node.cpp)
target_link_libraries(tools_node ${catkin_LIBRARIES})
add_dependencies(tools_node rovi_gencpp)

#target_link_libraries(grid_node
#   ${catkin_LIBRARIES}
#)

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_stst.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
