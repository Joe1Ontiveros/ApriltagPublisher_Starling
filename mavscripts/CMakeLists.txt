cmake_minimum_required(VERSION 2.8.3)
project(mavscripts)

# Find catkin and any required packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  mavros
  geometry_msgs
  std_msgs
  tf2_ros
  tf2
)

# Declare the package
catkin_package(
  CATKIN_DEPENDS roscpp mavros geometry_msgs std_msgs tf2_ros tf2
)

# Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Add the executable for mavros_apriltag_node
add_executable(mavros_apriltag_node src/mavros_apriltag_node.cpp)

# Link libraries
target_link_libraries(mavros_apriltag_node ${catkin_LIBRARIES})

# Add dependencies
add_dependencies(mavros_apriltag_node ${catkin_EXPORTED_TARGETS})

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Mark executables and/or libraries for installation
install(TARGETS
  mavros_apriltag_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)