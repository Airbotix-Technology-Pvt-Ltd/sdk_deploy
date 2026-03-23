cmake_minimum_required(VERSION 3.14)
project(isaac_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -O2)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(drdds REQUIRED)

include_directories(include)

add_executable(state_bridge_node
    src/state_bridge_node.cpp
)
ament_target_dependencies(state_bridge_node
    rclcpp
    sensor_msgs
    drdds
)

add_executable(cmd_bridge_node
    src/cmd_bridge_node.cpp
)
ament_target_dependencies(cmd_bridge_node
    rclcpp
    sensor_msgs
    drdds
)

install(TARGETS
    state_bridge_node
    cmd_bridge_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
