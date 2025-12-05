cmake_minimum_required(VERSION 3.5)
project(k100_motion_planning)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rviz_visual_tools REQUIRED)
find_package(control_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS chrono system filesystem)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

find_package(rosidl_default_generators REQUIRED)

# ===========================================
# 生成服务接口
# ===========================================
rosidl_generate_interfaces(${PROJECT_NAME}
  srv/PlanJointGoal.srv
  srv/PlayRosbag.srv
  srv/ComputeIK.srv
  srv/PlanPoseGoal.srv
  DEPENDENCIES geometry_msgs
)

include_directories(include)

# ===========================================
# 生成静态/动态库
# ===========================================
add_library(${PROJECT_NAME}_lib
  src/motion_planning.cpp
)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp pluginlib 
  moveit_core 
  moveit_ros_planning_interface 
  moveit_visual_tools 
  moveit_msgs 
  geometry_msgs
  sensor_msgs
  control_msgs
  rclcpp_action
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# === 新 recommended API：typesupport target ===
rosidl_get_typesupport_target(ts_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)
target_link_libraries(${PROJECT_NAME}_lib ${ts_target})


# ===========================================
# 可执行节点
# ===========================================
add_executable(k100_motion_planning_node
  src/main.cpp
  src/k100_motion_planning.cpp
)

target_link_libraries(k100_motion_planning_node
  ${PROJECT_NAME}_lib
  curl
  ${YAML_CPP_LIBRARIES}
)

target_include_directories(k100_motion_planning_node PRIVATE
  ${YAML_CPP_INCLUDE_DIRS}
)

ament_target_dependencies(k100_motion_planning_node 
  rclcpp moveit_core 
  moveit_ros_planning_interface 
  moveit_visual_tools 
  moveit_msgs 
  geometry_msgs 
  pluginlib
  rosbag2_cpp
  rosbag2_storage
  controller_manager_msgs
)

# === 绑定 typesupport ===
target_link_libraries(k100_motion_planning_node ${ts_target})


# ===========================================
# 安装
# ===========================================
install(TARGETS
  ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  k100_motion_planning_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(FILES
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
