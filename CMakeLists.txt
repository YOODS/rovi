cmake_minimum_required(VERSION 2.8.3)
project(rovi)

EXECUTE_PROCESS(COMMAND arch COMMAND tr -d '\n' OUTPUT_VARIABLE ARCH)
message(STATUS "Architecture: ${ARCH}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  stereo_msgs
  cv_bridge
  tf2_ros
  message_generation
  image_geometry
)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

add_service_files(
  FILES
  ImageFilter.srv
  GenPC.srv
)

add_message_files(
  FILES
  Floats.msg
  StringArray.msg
)

#catkin_python_setup()

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
  stereo_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs stereo_msgs cv_bridge tf2_ros image_geometry
  DEPENDS OpenCV
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

link_directories(
  lib/${ARCH}
)

add_definitions("-std=c++11")

#add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/rovi.cpp
#	src/libs/CircleCalibBoard.cpp
#)

#add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(remap_node src/remap_node.cpp)
target_link_libraries(remap_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(remap_node rovi_gencpp)

add_executable(genpc_node src/genpc_node.cpp)
target_link_libraries(genpc_node ${catkin_LIBRARIES} yds3d ${OpenCV_LIBRARIES})
add_dependencies(genpc_node rovi_gencpp)

add_executable(grid_node src/grid_node.cpp)
target_link_libraries(grid_node ${catkin_LIBRARIES} yds3d ${OpenCV_LIBRARIES})
add_dependencies(grid_node rovi_gencpp)

add_executable(floats2pc src/floats2pc.cpp)
target_link_libraries(floats2pc ${catkin_LIBRARIES})
add_dependencies(floats2pc rovi_gencpp)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
#install(PROGRAMS
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

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
