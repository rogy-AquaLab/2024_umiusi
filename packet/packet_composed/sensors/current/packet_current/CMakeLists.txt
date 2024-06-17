cmake_minimum_required(VERSION 3.8)
project(packet_current)

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

 add_executable(Current
  src/current_main.cpp
  src/current.cpp
 )

 ament_target_dependencies(Current
   rclcpp
   packet_interfaces
 )

 target_include_directories(Current
   PUBLIC
     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
     $<INSTALL_INTERFACE:include>
 )
 install(TARGETS
     Current
   DESTINATION lib/${PROJECT_NAME}
 )

ament_package()
