cmake_minimum_required(VERSION 3.8)
project(trapezoidal_acc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trapezoidal_acc REQUIRED)
find_package(std_msgs REQUIRED)
find_package(power_map_msg REQUIRED)

 add_executable(Trapezoid
  src/trapezoid_main.cpp
  src/trapezoid.cpp
 )

 ament_target_dependencies(Trapezoid
   rclcpp
   std_msgs
   power_map_msg
 )

 target_include_directories(Trapezoid
   PUBLIC
     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
     $<INSTALL_INTERFACE:include>
 )
 install(TARGETS
     Trapezoid
   DESTINATION lib/${PROJECT_NAME}
 )

ament_package()
