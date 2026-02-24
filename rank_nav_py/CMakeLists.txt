cmake_minimum_required(VERSION 3.5)
project(rank_nav_py)

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
find_package(rank_nav_msgs REQUIRED)

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

# Install the script
install(PROGRAMS
  scripts/rank_nav.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install the VLM_Rank_Nav library module
install(DIRECTORY VLM_Rank_Nav/
  DESTINATION lib/${PROJECT_NAME}/VLM_Rank_Nav
  PATTERN "cabot_run.py" EXCLUDE
  PATTERN "cabot_run_raw.py" EXCLUDE
  PATTERN "launch.py" EXCLUDE
  PATTERN ".git" EXCLUDE
  PATTERN "__pycache__" EXCLUDE
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
