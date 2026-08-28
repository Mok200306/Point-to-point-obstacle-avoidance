cmake_minimum_required(VERSION 3.8)
project(nav2_mppi_prediction_critic)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(nav2_mppi_controller REQUIRED)
find_package(oracle_dynamic_nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(nav2_mppi_prediction_critic SHARED
  src/prediction_critic.cpp
)

add_executable(prediction_critic_offline_test
  test/prediction_critic_offline_test.cpp
)
target_include_directories(prediction_critic_offline_test PRIVATE include)
target_compile_options(prediction_critic_offline_test PRIVATE -Wall -Wextra -Wpedantic)

target_include_directories(nav2_mppi_prediction_critic PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(nav2_mppi_prediction_critic
  nav2_mppi_controller
  oracle_dynamic_nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

target_compile_options(nav2_mppi_prediction_critic PRIVATE -Wall -Wextra -Wpedantic)

install(TARGETS nav2_mppi_prediction_critic
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS prediction_critic_offline_test
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES prediction_critics.xml
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(nav2_mppi_controller prediction_critics.xml)

ament_export_dependencies(
  nav2_mppi_controller
  oracle_dynamic_nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_prediction_grid_sampler
    test/test_prediction_grid_sampler.cpp)
  target_include_directories(test_prediction_grid_sampler PRIVATE include)
  if(TARGET test_prediction_grid_sampler)
    target_compile_options(test_prediction_grid_sampler PRIVATE -Wall -Wextra -Wpedantic)
  endif()
endif()

ament_package()
