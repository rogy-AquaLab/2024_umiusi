cmake_minimum_required(VERSION 3.8)
project(packet_composed)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(packet_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

 add_executable(Composed
  src/composed_main.cpp
  src/composed.cpp
 )

 ament_target_dependencies(Composed
   rclcpp
   std_msgs
   sensor_msgs
   packet_interfaces
 )
 
 set_property(TARGET Composed PROPERTY CXX_STANDARD 17)

 target_include_directories(Composed
   PUBLIC
     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
     $<INSTALL_INTERFACE:include>
 )
 install(TARGETS
     Composed
   DESTINATION lib/${PROJECT_NAME}
 )

ament_package()

