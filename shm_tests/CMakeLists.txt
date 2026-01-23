cmake_minimum_required(VERSION 3.8)
project(shm_tests)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FixedData128B.msg"
  "msg/FixedData1KB.msg"
  "msg/FixedData15KB.msg"
)

# dynamic size executables
add_executable(shm_test_dynamic_size_talker src/shm_test_dynamic_size_talker.cpp)
ament_target_dependencies(shm_test_dynamic_size_talker rclcpp std_msgs)

add_executable(shm_test_dynamic_size_listener src/shm_test_dynamic_size_listener.cpp)
ament_target_dependencies(shm_test_dynamic_size_listener rclcpp std_msgs)

# large msgs executables
add_executable(shm_test_large_msgs_talker src/shm_test_large_msgs_talker.cpp)
ament_target_dependencies(shm_test_large_msgs_talker rclcpp std_msgs)

add_executable(shm_test_large_msgs_listener src/shm_test_large_msgs_listener.cpp)
ament_target_dependencies(shm_test_large_msgs_listener rclcpp std_msgs)

# many topics executables
add_executable(shm_test_many_topics_talker src/shm_test_many_topics_talker.cpp)
ament_target_dependencies(shm_test_many_topics_talker rclcpp std_msgs)
rosidl_target_interfaces(shm_test_many_topics_talker ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(shm_test_many_topics_listener src/shm_test_many_topics_listener.cpp)
ament_target_dependencies(shm_test_many_topics_listener rclcpp std_msgs)
rosidl_target_interfaces(shm_test_many_topics_listener ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  shm_test_dynamic_size_talker
  shm_test_dynamic_size_listener
  shm_test_large_msgs_talker
  shm_test_large_msgs_listener
  shm_test_many_topics_talker
  shm_test_many_topics_listener
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/two_container_shm_performance_test.py
  scripts/shm_performance_test_utils.py
  scripts/shm_performance_test.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()