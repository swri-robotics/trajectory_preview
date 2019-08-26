cmake_minimum_required(VERSION 2.8.3)
project(trajectory_preview)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rviz
  sensor_msgs
  trajectory_msgs
  tesseract_msgs
)

find_package(Qt5 REQUIRED COMPONENTS
  Core
  Widgets
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}_widget
    ${PROJECT_NAME}_panel
  CATKIN_DEPENDS
    rviz
    sensor_msgs
    trajectory_msgs
    tesseract_msgs
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
)

qt5_wrap_ui(UI_MOC
  ui/trajectory_preview_widget.ui
)

qt5_wrap_cpp(WIDGET_MOC
  include/${PROJECT_NAME}/trajectory_preview_widget.h
  src/trajectory_preview_impl.h
)

qt5_wrap_cpp(PANEL_MOC
  include/${PROJECT_NAME}/trajectory_preview_panel.h
)

# Widget Library
add_library(${PROJECT_NAME}_widget
  src/trajectory_preview_widget.cpp
  ${UI_MOC}
  ${WIDGET_MOC}
)
target_link_libraries(${PROJECT_NAME}_widget
  ${catkin_LIBRARIES}
  Qt5::Widgets
)

# Panel Library
add_library(${PROJECT_NAME}_panel
  src/trajectory_preview_panel.cpp
  ${PANEL_MOC}
)
target_link_libraries(${PROJECT_NAME}_panel
  ${catkin_LIBRARIES}
  Qt5::Widgets
  ${PROJECT_NAME}_widget
)

#############
## Install ##
#############

install(
  TARGETS
    ${PROJECT_NAME}_widget
    ${PROJECT_NAME}_panel
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
