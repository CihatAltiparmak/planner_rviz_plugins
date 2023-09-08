cmake_minimum_required(VERSION 3.5)
project(planner_rviz_plugins)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_FLAGS "-Wl,--no-undefined")

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
## This setting causes Qt's "MOC" generation to happen automatically.
set(CMAKE_AUTOMOC ON)

set(INCLUDE_FILES_QT
  include/planner_rviz_plugins/trajectory_display.hpp
)

qt5_wrap_cpp(MOC_FILES ${INCLUDE_FILES_QT})

## Here we specify the list of source files.
## The generated MOC files are included automatically as headers.
set(SRC_FILES
  src/trajectory_display.cpp
  src/trajectory_visual.cpp
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED ${SRC_FILES} ${MOC_FILES})
ament_target_dependencies(${PROJECT_NAME}
  "geometry_msgs"
  "rclcpp"
  "rviz_common"
  "rviz_rendering")

target_include_directories(${PROJECT_NAME} PUBLIC
  ${Qt5Widgets_INCLUDE_DIRS})

## Here we export the plugins and meshes so they can be found by RViz at runtime.
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

ament_export_dependencies(
  geometry_msgs
  rclcpp
  rviz_common
  rviz_rendering)

install(
  FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

## END_TUTORIAL

ament_export_dependencies(
  geometry_msgs
  rclcpp
  rviz_common
  rviz_rendering)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
