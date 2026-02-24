cmake_minimum_required(VERSION 3.5)
project(sngnn_py)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(people_msgs REQUIRED)

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

# Install the script
install(PROGRAMS
  scripts/sngnn_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install the SNGNN2D-v2 library module
install(DIRECTORY SNGNN2D-v2/
  DESTINATION lib/${PROJECT_NAME}/SNGNN2D-v2
  PATTERN ".git" EXCLUDE
  PATTERN "__pycache__" EXCLUDE
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
