cmake_minimum_required(VERSION 3.8)
project(imshow)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}  # OpenCVの追加
)

# Create executable
add_executable(imshow src/imshow.cpp src/main.cpp)

# Link dependencies
ament_target_dependencies(imshow rclcpp sensor_msgs cv_bridge image_transport)

target_link_libraries(imshow ${OpenCV_LIBRARIES})

# Install targets
install(TARGETS
  imshow
  DESTINATION lib/${PROJECT_NAME})

ament_package()
