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

# `composed`executableを作る
 add_executable(composed
   src/composed_main.cpp
  src/composed.cpp
 )

 # `composed`executableの依存関係
 ament_target_dependencies(composed
   rclcpp       # ROS2標準ライブラリ
   std_msgs     # ROS2が用意している標準的なmessage型の定義を含むライブラリ
 )

 # `composed`executableが参照するヘッダファイルがあるフォルダー
 target_include_directories(composed
   PUBLIC
     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
     $<INSTALL_INTERFACE:include>
 )

 # `listener`executableの置き場所
 # 各executableの置き場所
 install(TARGETS
    composed
  DESTINATION lib/${PROJECT_NAME}
 )
 
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
