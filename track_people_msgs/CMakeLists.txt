cmake_minimum_required(VERSION 3.5)
project(track_people_msgs)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_copyright REQUIRED)
#  find_package(ament_cmake_cpplint REQUIRED)
#  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
#  find_package(ament_cmake_uncrustify REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)

  set(MAX_LINE_LENGTH 200)
  ament_copyright()
#  ament_cpplint(MAX_LINE_LENGTH ${MAX_LINE_LENGTH})
#  ament_flake8(MAX_LINE_LENGTH ${MAX_LINE_LENGTH})
  ament_lint_cmake()
#  ament_uncrustify(MAX_LINE_LENGTH ${MAX_LINE_LENGTH})
  ament_xmllint()
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BoundingBox.msg"
  "msg/TrackedBox.msg"
  "msg/TrackedBoxes.msg"
  DEPENDENCIES geometry_msgs sensor_msgs std_msgs
)

ament_package()
