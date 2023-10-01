cmake_minimum_required(VERSION 3.8)
project(nav_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_CONFIG_PATH ${CMAKE_MODULE_PATH}  "${CMAKE_CURRENT_LIST_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CONFIG_PATH}")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(action_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(ZMQ)
if(ZMQ_FOUND)
    message(STATUS "ZeroMQ found.")
    add_definitions(-DZMQ_FOUND)
else()
  message(WARNING "ZeroMQ NOT found. Not including PublisherZMQ.")
endif()

set(dependencies
    rclcpp
    rclcpp_lifecycle
    rclcpp_action
    behaviortree_cpp_v3
    action_msgs
    lifecycle_msgs
    geometry_msgs
    sensor_msgs
    ament_index_cpp
)

include_directories(include ${ZMQ_INCLUDE_DIRS})

add_library(nav_control_recharge_bt_node SHARED src/nav_control/Recharge.cpp)
add_library(nav_control_battery_checker_bt_node SHARED src/nav_control/BatteryChecker.cpp)
add_library(nav_control_get_line_bt_node SHARED src/nav_control/GetLine.cpp)
add_library(nav_control_nav_line_bt_node SHARED src/nav_control/NavLine.cpp)
add_library(nav_control_change_line_bt_node SHARED src/nav_control/ChangeLine.cpp)

list(APPEND plugin_libs
  nav_control_battery_checker_bt_node
  nav_control_recharge_bt_node
  nav_control_get_line_bt_node
  nav_control_nav_line_bt_node
  nav_control_change_line_bt_node
)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_executable(nav_main src/nav_main.cpp)
ament_target_dependencies(nav_main ${dependencies})
target_link_libraries(nav_main ${ZMQ_LIBRARIES})

install(TARGETS
  ${plugin_libs}
  nav_main
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(
  DIRECTORY
    launch generated behavior_tree_xml segmentation_model
  DESTINATION
    share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

endif()

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

ament_package()
