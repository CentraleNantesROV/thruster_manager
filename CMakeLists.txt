cmake_minimum_required(VERSION 3.5)
project(thruster_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()


# thruster manager as a library / composition node
add_library(thruster_manager SHARED src/thruster_manager.cpp src/thruster_manager_node.cpp)
target_include_directories(thruster_manager PUBLIC include ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(thruster_manager rclcpp rclcpp_lifecycle urdf tf2_ros sensor_msgs geometry_msgs kdl_parser rclcpp_components)

# rviz_bridge to display thruster wrenches
add_executable(publish_wrenches src/publish_wrenches.cpp)
target_include_directories(publish_wrenches PUBLIC include ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(publish_wrenches thruster_manager)

#if(NOT WIN32)
  #ament_environment_hooks(
    #"${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}"
    #)
#endif()

# auto-executable from node
rclcpp_components_register_node(thruster_manager
  PLUGIN "thruster_manager::ThrusterManagerNode"
  EXECUTABLE thruster_manager_node)

install(TARGETS thruster_manager publish_wrenches
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

# allow using the class in another package
# legacy include dir + nested since Humble
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

ament_export_libraries(thruster_manager)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp_lifecycle rclcpp_components)

ament_package()
