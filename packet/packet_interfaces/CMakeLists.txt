cmake_minimum_required(VERSION 3.8)
project(packet_interfaces)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    msg/Composed.msg
    msg/Current.msg
    msg/Depth.msg
    msg/Flex.msg
    msg/LedColor.msg
    msg/NucleoState.msg
    msg/Power.msg
    msg/Voltage.msg
    DEPENDENCIES std_msgs sensor_msgs
)

ament_package()
