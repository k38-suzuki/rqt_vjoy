cmake_minimum_required(VERSION 3.0.2)
project(rqt_vjoy)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rqt_gui
  rqt_gui_cpp
  sensor_msgs
  std_msgs
)
find_package(Qt5 COMPONENTS Widgets REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rqt_vjoy
  CATKIN_DEPENDS roscpp rqt_gui rqt_gui_cpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
link_directories(${catkin_LIBRARY_DIRS})

set(sources
  src/${PROJECT_NAME}/my_plugin.cpp
  src/${PROJECT_NAME}/my_widget.cpp
  src/${PROJECT_NAME}/joystick_interface.cpp
  src/${PROJECT_NAME}/joy_widget.cpp
)

set(headers
  include/${PROJECT_NAME}/my_plugin.h
  include/${PROJECT_NAME}/my_widget.h
  include/${PROJECT_NAME}/joystick_interface.h
  include/${PROJECT_NAME}/joy_widget.h
)

qt5_wrap_cpp(rqt_vjoy_moc ${headers})

add_library(${PROJECT_NAME} ${sources} ${rqt_vjoy_moc})

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} Qt5::Widgets)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(FILES
  plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
