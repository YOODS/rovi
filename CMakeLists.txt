cmake_minimum_required(VERSION 2.8.3)
project(rovi)

EXECUTE_PROCESS(COMMAND arch COMMAND tr -d '\n' OUTPUT_VARIABLE ARCH)
message(STATUS "Architecture: ${ARCH}")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  stereo_msgs
  cv_bridge
  tf2_ros
  pcl_ros
  pcl_conversions
  message_generation
  image_geometry
)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

#2020/09/09 add by hato ---------- start ----------
find_package(Aravis REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(GLIB REQUIRED glib-2.0)
#2020/09/09 add by hato ----------  end  ----------

#2020/09/18 add by hato ---------- start ----------
find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
  message(STATUS "OPENMP FOUND")
  set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})  # or if you use C: ${OpenMP_C_FLAGS}
  set(OpenMP_LIBS gomp)
endif()
#2020/09/18 add by hato ----------  end  ----------

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
#2020/09/18 add by hato ---------- start ----------
  OpenMP
#2020/09/18 add by hato ----------  end  ----------
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
#2020/09/09 add by hato ---------- start ----------
  ${ARAVIS_INCLUDE_PATH}
  ${GLIB_INCLUDE_DIRS}
#2020/09/09 add by hato ----------  end  ----------
)

link_directories(
  lib/${ARCH}
)

add_definitions("-std=c++11")

#2020/08/25 add by hato ---------- start ----------
add_definitions("-DYAML_PARAM")
#2020/08/25 add by hato ----------  end  ----------

#add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/rovi.cpp
#	src/libs/CircleCalibBoard.cpp
#)

#add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(remap_node src/remap_node.cpp)
target_link_libraries(remap_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(remap_node rovi_gencpp)

#2020/09/17 modified by hato ----------------- start ------------------
#add_executable(genpc_node src/genpc_node.cpp)
add_executable(genpc_node src/genpc_node.cpp src/YPCData.cpp src/ElapsedTimer.cpp)
#target_link_libraries(genpc_node ${catkin_LIBRARIES} yds3d ${OpenCV_LIBRARIES})
target_link_libraries(genpc_node ${catkin_LIBRARIES} yds3d yaml-cpp ${OpenCV_LIBRARIES})
#2020/09/17 modified by hato -----------------  end ------------------
add_dependencies(genpc_node rovi_gencpp)

add_executable(grid_node src/grid_node.cpp)
#2020/09/17 modified by hato ----------------- start ------------------
#target_link_libraries(grid_node ${catkin_LIBRARIES} yds3d ${OpenCV_LIBRARIES})
target_link_libraries(grid_node ${catkin_LIBRARIES} yds3d ${OpenCV_LIBRARIES} ${OpenMP_LIBS})
#2020/09/17 modified by hato -----------------  end  ------------------
add_dependencies(grid_node rovi_gencpp)

add_executable(floats2pc src/floats2pc.cpp)
target_link_libraries(floats2pc ${catkin_LIBRARIES})
add_dependencies(floats2pc rovi_gencpp)

#2020/09/09 modified by hato ----------------- start ------------------
add_executable(ycam3d_node src/ycam3d_node.cpp src/Aravis.cpp src/CameraYCAM3D.cpp src/ElapsedTimer.cpp)
target_link_libraries(ycam3d_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${ARAVIS_LIBRARY} gobject-2.0 glib-2.0 )
add_dependencies(ycam3d_node rovi_gencpp)
#2020/09/09 modified by hato -----------------  end  ------------------

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
