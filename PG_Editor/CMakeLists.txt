cmake_minimum_required(VERSION 2.8.3)
project(pg_editor)

 
add_compile_options(-std=c++14)

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
  TransformationInfo.msg
  RelativeFramesInfo.msg
  RelativePoseInfo.msg
)

add_service_files(
  FILES
  AddFactor.srv
)


generate_messages(
  DEPENDENCIES
  std_msgs
  rideflux_msgs
  geometry_msgs
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

add_executable(pose_graph_example_node src/pose_graph_example_node.cpp)
target_link_libraries(pose_graph_example_node
  # ${rf_lib}
  ${catkin_LIBRARIES}
  ${MKL_LIBRARIES}
  # ${TBB_LIBRARIES}
)
add_dependencies(pose_graph_example_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
message(MKL_LIBRARIES: ${MKL_LIBRARIES})

add_executable(ndt_matcher_example_node2 src/ndt_matcher_example_node.cpp)
target_link_libraries(ndt_matcher_example_node2
  # ${rf_lib}
  ${catkin_LIBRARIES}
  ${MKL_LIBRARIES}
  # ${TBB_LIBRARIES}
)
add_dependencies(ndt_matcher_example_node2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(tf_visualizer_node2 src/tf_visualizer_node.cpp)
target_link_libraries(tf_visualizer_node2
  # ${rf_lib}
  ${catkin_LIBRARIES}
  ${MKL_LIBRARIES}
  # ${TBB_LIBRARIES}
)
add_dependencies(tf_visualizer_node2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(a_click_receiver_node src/a_click_receiver_node.cpp)
target_link_libraries(a_click_receiver_node
  # ${rf_lib}
  ${catkin_LIBRARIES}
  ${MKL_LIBRARIES}
  # ${TBB_LIBRARIES}
)
add_dependencies(a_click_receiver_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(read_imu_pose_data src/read_imu_pose_data.cpp)
target_link_libraries(read_imu_pose_data
  # ${rf_lib}
  ${catkin_LIBRARIES}
  ${MKL_LIBRARIES}
  # ${TBB_LIBRARIES}
)
add_dependencies(read_imu_pose_data ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})