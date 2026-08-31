cmake_minimum_required(VERSION 3.8)
project(rtabmap_tb3_nav)

find_package(ament_cmake REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_smac_planner REQUIRED)
find_package(pluginlib REQUIRED)

add_library(goal_line_smac_planner SHARED
  src/goal_line_smac_planner.cpp)
target_compile_features(goal_line_smac_planner PUBLIC cxx_std_17)
target_include_directories(goal_line_smac_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(goal_line_smac_planner
  nav2_core nav2_smac_planner pluginlib)
target_link_libraries(goal_line_smac_planner
  ${nav2_smac_planner_LIBRARIES})

pluginlib_export_plugin_description_file(nav2_core goal_line_smac_planner.xml)

install(
  DIRECTORY behavior_trees config launch worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS goal_line_smac_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  PROGRAMS
    scripts/send_goal.py
    scripts/explore_demo.py
    scripts/dynamic_obstacle_driver.py
    scripts/navigation_trial.py
    scripts/goal_line_visualizer.py
    scripts/multi_goal_trial.py
    scripts/multi_waypoint_trial.py
    scripts/map_padder.py
    scripts/real_robot_preflight.py
    scripts/real_navigation_trial.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
