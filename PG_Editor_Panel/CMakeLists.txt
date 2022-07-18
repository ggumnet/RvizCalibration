cmake_minimum_required(VERSION 2.8.3)
project(pg_editor_panel)

add_compile_options(-std=c++14)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rviz
  std_msgs
  std_srvs
)

catkin_package(
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

find_package(Qt5 COMPONENTS Core Widgets PATHS "/lib/libqt")

set(QT_LIBRARIES Qt5::Widgets)
include_directories(include/)
include_directories(${Qt5Widgets_INCLUDES})

qt5_wrap_cpp(QT_MOC include/pg_editor_panel/PGEditorPanel.h)
set(SOURCES src/PGEditorPanel.cpp  ${QT_MOC})


add_library(${PROJECT_NAME} ${SOURCES})
target_link_libraries(${PROJECT_NAME} ${rviz_DEFAULT_PLUGIN_LIBRARIES} ${QT_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})


install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)