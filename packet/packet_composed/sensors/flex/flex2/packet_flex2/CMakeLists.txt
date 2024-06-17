cmake_minimum_required(VERSION 3.8)
project(packet_flex2)

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

 add_executable(Flex2
  src/flex1_main.cpp
  src/flex1.cpp
 )

 ament_target_dependencies(Flex2
   rclcpp
   packet_interfaces
 )

 target_include_directories(Flex2
   PUBLIC
     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
     $<INSTALL_INTERFACE:include>
 )
 install(TARGETS
     Flex2
   DESTINATION lib/${PROJECT_NAME}
 )

ament_package()
