cmake_minimum_required(VERSION 3.5)
project(compressed_pointcloud_transport)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(PCL REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(compressed_pointcloud_interfaces REQUIRED)

## PCL library
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(
	include
  ${PCL_INCLUDE_DIRS}
)

add_executable(compress
  src/compress.cpp
)

ament_target_dependencies(compress rclcpp std_msgs compressed_pointcloud_interfaces)

# link libraries
target_link_libraries(compress
	${Boost_LIBRARY}
	${PCL_LIBRARIES}
)

add_executable(decompress
  src/decompress.cpp
)

ament_target_dependencies(decompress rclcpp std_msgs compressed_pointcloud_interfaces)

# link libraries
target_link_libraries(decompress
	${Boost_LIBRARY}
	${PCL_LIBRARIES}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS 
  compress
  decompress
DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY 
launch
config
DESTINATION share/${PROJECT_NAME})

ament_package()
