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

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_ros
  message_generation
  compressed_pointcloud_interfaces
)

# find dependencies
find_package(PCL REQUIRED)

catkin_package(CATKIN_DEPENDS
	roscpp rospy std_msgs message_runtime
	pcl_ros
  )

## PCL library
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(
	include
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(compressed_pointcloud_transport SHARED
  src/compressed_pointcloud_transport.cpp
)

# link libraries
target_link_libraries(compressed_pointcloud_transport
	${Boost_LIBRARY}
	${PCL_LIBRARIES}
)

add_executable(compress
  src/compress.cpp
)

target_link_libraries(compress
  ${catkin_LIBRARIES}
	${Boost_LIBRARY}
	${PCL_LIBRARIES}
  compressed_pointcloud_transport
)

add_executable(decompress
  src/decompress.cpp
)

target_link_libraries(decompress
  ${catkin_LIBRARIES}
	${Boost_LIBRARY}
	${PCL_LIBRARIES}
  compressed_pointcloud_transport
)

# add_executable(decompress
#   src/decompress.cpp
# )

# ament_target_dependencies(decompress rclcpp std_msgs compressed_pointcloud_interfaces)

# # link libraries
# target_link_libraries(decompress
# 	${Boost_LIBRARY}
# 	${PCL_LIBRARIES}
#   compressed_pointcloud_transport
# )

## Mark libraries for installation
# See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(TARGETS
  compress
  decompress
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )
