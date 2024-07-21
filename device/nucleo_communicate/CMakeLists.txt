cmake_minimum_required(VERSION 3.8)
project(nucleo_communicate)

set(CMAKE_CXX_STANDARD 20)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(packet_interfaces REQUIRED)

add_library(channel
    SHARED
    src/recv_data.cpp
    src/send_data.cpp
    src/channel.cpp
)

ament_target_dependencies(channel
    rclcpp
    rclcpp_components
    std_msgs
    packet_interfaces
)

target_include_directories(channel
    PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

rclcpp_components_register_node(
    channel
    PLUGIN "channel::Channel"
    EXECUTABLE nucleo-channel
)

install(
    TARGETS channel
    EXPORT channel_component
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# install(
#     DIRECTORY launch
#     DESTINATION share/${PROJECT_NAME}
# )

ament_package()
