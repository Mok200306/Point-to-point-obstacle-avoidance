cmake_minimum_required(VERSION 3.8)
project(rtabmap_tb3_nav)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/send_goal.py
    scripts/explore_demo.py
    scripts/navigation_trial.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
