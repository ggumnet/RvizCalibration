cmake_minimum_required(VERSION 2.8.3)
project(pg_editor)
 
add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  interactive_markers
  pcl_ros
  roscpp
  std_srvs
  message_generation
  tf2_ros
  pg_lib
  rf_geometry
  pointcloud_tools
  rideflux_msgs
  rf_nd_tree
  tf
  tf2_ros
  # rf_map_build_manager
)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
find_package(TBB)
find_package(MKL)
add_compile_options(-DMKL_ILP64)

find_package(OpenMP REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

find_package(PythonLibs 2.7)


# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -I/home/gw/rf_lib_ws/install/include")
# set(rf_lib "/home/gw/rf_lib_ws/install/lib/libNDT_GPU_LIB.so;/home/gw/rf_lib_ws/install/lib/libpg_lib.so;/home/gw/rf_lib_ws/install/lib/libpointcloud_tools.so;/home/gw/rf_lib_ws/install/lib/librf_nd_tree.so")
add_message_files(
  FILES
  RelativeFramesInfo.msg
  RelativePoseInfo.msg
  TransformInfo.msg
)

add_service_files(
  FILES
  AddFactor.srv
  GetNDTMatchingResult.srv
  GetPointcloud.srv
  GetImuPoseResult.srv
  SendConfiguration.srv
  TfBroadcastInfo.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  rideflux_msgs
  geometry_msgs
)

generate_dynamic_reconfigure_options(
  cfg/InitialConfiguration.cfg
  #...
)

catkin_package(
#  INCLUDE_DIRS include
 CATKIN_DEPENDS dynamic_reconfigure interactive_markers pcl_ros roscpp std_srvs tf2_ros message_generation message_runtime
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${MKL_INCLUDE_DIR}
  ${PYTHON_INCLUDE_DIRS}
)

SET(GCC_COVERAGE_LINK_FLAGS "-lstdc++fs")

add_executable(main_server src/main_server.cpp)
target_link_libraries(main_server
  # ${rf_lib}
  ${catkin_LIBRARIES}
  ${MKL_LIBRARIES}
  # ${TBB_LIBRARIES}
  ${GCC_COVERAGE_LINK_FLAGS}
)
add_dependencies(main_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
message(MKL_LIBRARIES: ${MKL_LIBRARIES})


add_executable(ndt_matching_server src/ndt_matching_server.cpp)
target_link_libraries(ndt_matching_server
  # ${rf_lib}
  ${catkin_LIBRARIES}
  ${MKL_LIBRARIES}
  # ${TBB_LIBRARIES}
)
add_dependencies(ndt_matching_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(data_reader_server src/data_reader_server.cpp)
target_link_libraries(data_reader_server
  # ${rf_lib}
  ${catkin_LIBRARIES}
  ${MKL_LIBRARIES}
  # ${TBB_LIBRARIES}
)
add_dependencies(data_reader_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(tf_broadcaster_node src/tf_broadcaster_node.cpp)
target_link_libraries(tf_broadcaster_node
  # ${rf_lib}
  ${catkin_LIBRARIES}
  ${MKL_LIBRARIES}
  # ${TBB_LIBRARIES}
)
add_dependencies(tf_broadcaster_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(lidar_imu_graph_calibration_example_node src/lidar_imu_graph_calibration_example_node.cpp)
target_link_libraries(lidar_imu_graph_calibration_example_node
  # ${rf_lib}
  ${catkin_LIBRARIES}
  ${MKL_LIBRARIES}
  # ${TBB_LIBRARIES}
)
add_dependencies(lidar_imu_graph_calibration_example_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

