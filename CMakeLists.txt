# More details at: http://wiki.ros.org/catkin/CMakeLists.txt

cmake_minimum_required(VERSION 2.8.3)
project(vision)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  image_transport
  sensor_msgs
  cv_bridge
  message_generation
  freenect_camera
  freenect_launch
)

add_definitions(-std=c++11)

find_package(Boost REQUIRED COMPONENTS system)

find_package(freenect_launch REQUIRED)

find_package(freenect_camera REQUIRED)

add_message_files(DIRECTORY msg FILES
  VisionMessage.msg)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
  INCLUDE_DIRS
  LIBRARIES
)

include_directories(
  include/
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${SDFormat_INCLUDE_DIRS}
)

link_directories(
  ${Boost_LIBRARY_DIRS}
  ${SDFormat_LIBRARY_DIRS}
)

add_executable(vision_node
  src/vision/vision_node.cpp
  src/vision/vision.cpp
  src/vision/raw_image.cpp
  src/vision/algorithm.cpp
  src/vision/segmenter.cpp
  src/vision/segmentation_algorithm.cpp
  src/vision/identifier.cpp
  src/vision/identification_algorithm.cpp
  src/vision/tracker.cpp
  src/vision/tracked_object.cpp
  src/vision/config_parser.cpp
  src/vision/algorithm_factory.cpp
  src/vision/vision_gui.cpp
  src/vision/dummy_segmentation_algorithm.cpp
  src/vision/dummy_identification_algorithm.cpp)
add_dependencies(vision_node unball_gencpp)
target_link_libraries(vision_node ${catkin_LIBRARIES})

add_executable(dummy_image
  src/dummy_camera/dummy_image.cpp)
add_dependencies(dummy_image unball_gencpp)
target_link_libraries(dummy_image ${catkin_LIBRARIES})
