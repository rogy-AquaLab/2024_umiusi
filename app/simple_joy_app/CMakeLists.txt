cmake_minimum_required(VERSION 3.8)
project(simple_joy_app)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)

add_library(simple_joy_app
    SHARED
    src/app.cpp
)

ament_target_dependencies(simple_joy_app
    rclcpp
    sensor_msgs
    rclcpp_components
)

target_include_directories(simple_joy_app
    PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

rclcpp_components_register_node(
    simple_joy_app
    PLUGIN "app::App"
    EXECUTABLE app
)

install(
    TARGETS simple_joy_app
    EXPORT export_simple_joy_app_component
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

ament_package()
