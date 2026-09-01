cmake_minimum_required(VERSION 3.12)
project(bsp)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")

  add_compile_options(-Wall -Wextra -Wpedantic)

endif()



# ==============================================================================

# 依赖寻包

# ==============================================================================

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)

find_package(rclcpp_lifecycle REQUIRED)

find_package(std_msgs REQUIRED)

find_package(sensor_msgs REQUIRED)

find_package(std_srvs REQUIRED)

find_package(trajectory_msgs REQUIRED)

find_package(lifecycle_msgs REQUIRED)

find_package(cv_bridge REQUIRED)

find_package(OpenCV REQUIRED)

find_package(Threads REQUIRED)



# hal 包提供 hal::msg / hal::srv 自定义接口头文件

find_package(hal REQUIRED)



include_directories(include)



# ==============================================================================

# BSP 层节点

# ==============================================================================



# 1. 通信网关（HAL 状态 -> UDP 地面站；地面站 UDP 指令 -> ROS 话题）

add_executable(bsp_comm_node src/bsp_comm_node.cpp)

ament_target_dependencies(bsp_comm_node

  rclcpp rclcpp_lifecycle std_msgs sensor_msgs cv_bridge hal)

target_link_libraries(bsp_comm_node ${OpenCV_LIBS})



# 2. 闭环运动控制（6-DOF 前馈 + PID + 控制分配）

add_executable(bsp_motioncontrol_node src/bsp_motioncontrol_node.cpp)

ament_target_dependencies(bsp_motioncontrol_node

  rclcpp rclcpp_lifecycle std_msgs lifecycle_msgs hal)



# 3. 开环遥控节点

add_executable(bsp_remotecontrol_node src/bsp_remotecontrol_node.cpp)

ament_target_dependencies(bsp_remotecontrol_node

  rclcpp rclcpp_lifecycle std_msgs lifecycle_msgs hal)



# 4. 机械臂轨迹插值节点

add_executable(bsp_arm_trajectory_node src/bsp_arm_trajectory_node.cpp)

ament_target_dependencies(bsp_arm_trajectory_node rclcpp sensor_msgs trajectory_msgs)


# 5. 监测节点

add_executable(bsp_monitor_node src/bsp_monitor_node.cpp)

ament_target_dependencies(bsp_monitor_node  rclcpp  lifecycle_msgs  std_msgs)
# ==============================================================================

# 安装与导出

# ==============================================================================

install(TARGETS

  bsp_comm_node

  bsp_motioncontrol_node

  bsp_remotecontrol_node

  bsp_arm_trajectory_node
  
  bsp_monitor_node

  DESTINATION lib/${PROJECT_NAME}

)



install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

install(DIRECTORY include/ DESTINATION include/)



ament_export_dependencies(rosidl_default_runtime)



ament_package() 




