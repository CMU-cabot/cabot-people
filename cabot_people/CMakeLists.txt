cmake_minimum_required(VERSION 3.5)
project(cabot_people)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

install(PROGRAMS
  script/framos_initialize_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
