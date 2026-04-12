cmake_minimum_required(VERSION 3.8)
project(rc_car_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf config launch worlds rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
