cmake_minimum_required(VERSION 2.8.3)
project(nodelet_example)

# Find required ROS packages and other dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  std_msgs
)

# Declare that this catkin package's runtime dependencies
catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp nodelet std_msgs
)

# Include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Add a library with the same name as the project
add_library(${PROJECT_NAME}
  src/Nodelets.cpp
)

# Link libraries
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# Make sure configure headers are built before any node using this package
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Install the plugin description XML file
install(FILES nodelet_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Install the library
install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)



