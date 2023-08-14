cmake_minimum_required(VERSION 3.5.0)
project(trajectory_preview)

add_compile_options(-std=c++14)

find_package(Qt5 REQUIRED COMPONENTS Widgets)

set(ROS2_DEPS
  rviz_common
  sensor_msgs
  trajectory_msgs
  pluginlib
)

find_package(ament_cmake REQUIRED)
foreach(dep ${ROS2_DEPS})
  find_package(${dep} REQUIRED)
endforeach()

###########
## Build ##
###########
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
add_library(${PROJECT_NAME}_widget SHARED
  src/trajectory_preview_widget.cpp
  ${UI_MOC}
  ${WIDGET_MOC}
)
target_include_directories(${PROJECT_NAME}_widget PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
target_include_directories(${PROJECT_NAME}_widget PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>"
)
target_link_libraries(${PROJECT_NAME}_widget
  Qt5::Widgets
)
ament_target_dependencies(${PROJECT_NAME}_widget ${ROS2_DEPS})

# Panel Library
add_library(${PROJECT_NAME}_panel SHARED
  src/trajectory_preview_panel.cpp
  ${PANEL_MOC}
)
target_link_libraries(${PROJECT_NAME}_panel
  Qt5::Widgets
  ${PROJECT_NAME}_widget
)

#############
## Install ##
#############

# Install the headers
install(DIRECTORY include/${PROJECT_NAME} DESTINATION include)

# Install the library(ies)
install(TARGETS ${PROJECT_NAME}_widget ${PROJECT_NAME}_panel
        EXPORT ${PROJECT_NAME}-targets DESTINATION lib)

install(FILES
  plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

ament_export_targets(${PROJECT_NAME}-targets HAS_LIBRARY_TARGET)
ament_export_dependencies(${ROS2_DEPS})
ament_package()
