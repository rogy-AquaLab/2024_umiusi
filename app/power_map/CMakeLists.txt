cmake_minimum_required(VERSION 3.8)
project(power_map)

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
find_package(packet_interfaces REQUIRED)
find_package(power_map_msg REQUIRED)

add_library(power_map
    SHARED
    src/power_map.cpp
    src/placement.cpp
)

ament_target_dependencies(power_map
    rclcpp
    rclcpp_components
    packet_interfaces
    power_map_msg
)

target_include_directories(power_map
    PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

rclcpp_components_register_node(
    power_map
    PLUGIN "power_map::PowerMap"
    EXECUTABLE power-map
)

install(
    TARGETS power_map
    EXPORT export_power_map_component
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(
    DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
