cmake_minimum_required(VERSION 3.0)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Setup the project.
# Its name is defined to be the controller directory name.
get_filename_component(PROJECT ${CMAKE_SOURCE_DIR} NAME)
project(${PROJECT})

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs apa_msg)
message(${catkin_INCLUDE_DIRS})
find_package(PCL REQUIRED COMPONENTS common)
 
set(webots_INCLUDE_DIR $ENV{WEBOTS_HOME}/include/controller/cpp)
file(GLOB webots_LIBRARIES $ENV{WEBOTS_HOME}/lib/controller/${CMAKE_SHARED_LIBRARY_PREFIX}*${CMAKE_SHARED_LIBRARY_SUFFIX})

include_directories(
  ${webots_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS})

add_library(webots_ros_bridge webots_ros_publisher.cpp)
target_link_libraries(webots_ros_bridge ${webots_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# Setup the target executable.
add_executable(${PROJECT} autonomous_vehicle.cpp)
target_link_libraries(${PROJECT} webots_ros_bridge ${webots_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# Copy the target executable at the right location.
add_custom_command(TARGET ${PROJECT} POST_BUILD COMMAND ${CMAKE_COMMAND} -E
  copy ${CMAKE_BINARY_DIR}/${PROJECT} ${CMAKE_SOURCE_DIR}
)