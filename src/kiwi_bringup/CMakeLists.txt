cmake_minimum_required(VERSION 3.8)
project(kiwi_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config docs launch rviz
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(
  FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
