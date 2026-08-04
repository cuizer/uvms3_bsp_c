cmake_minimum_required(VERSION 3.8)
project(bsp_comm_cpp_pkg)

# ✅ C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(uvms3_test REQUIRED)

# executable
add_executable(bsp_comm_node src/bsp_comm_node.cpp)

ament_target_dependencies(bsp_comm_node
  rclcpp
  rclcpp_lifecycle
  std_msgs
  sensor_msgs
  cv_bridge
  uvms3_test
)

target_link_libraries(bsp_comm_node
  ${OpenCV_LIBS}
)

# install
install(TARGETS
  bsp_comm_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
