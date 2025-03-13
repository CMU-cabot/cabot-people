cmake_minimum_required(VERSION 3.5)
project(cabot_people)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wfatal-errors)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(
  include
)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

set(dependencies
  rclcpp
  sensor_msgs
  std_srvs
  tf2_ros
  tf2_geometry_msgs
)

## framos_initialize_node
add_executable(framos_initialize_node
  src/framos_initialize_node.cpp
)

ament_target_dependencies(framos_initialize_node
  ${dependencies}
)

install(TARGETS
  framos_initialize_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
