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

file(GLOB_RECURSE vision_node_sources src/vision/*.cpp)
add_executable(vision_node ${vision_node_sources})
add_dependencies(vision_node unball_gencpp)
target_link_libraries(vision_node ${catkin_LIBRARIES})

add_executable(dummy_image
  src/dummy_camera/dummy_image.cpp)
add_dependencies(dummy_image unball_gencpp)
target_link_libraries(dummy_image ${catkin_LIBRARIES})

file(GLOB_RECURSE calibration_node_sources src/calibration/*.cpp)
add_executable(calibration_node ${calibration_node_sources})
add_dependencies(calibration_node unball_gencpp)
target_link_libraries(calibration_node ${catkin_LIBRARIES})

add_executable(save_kinect_video
  src/camera/save_kinect_video.cpp)
add_dependencies(save_kinect_video unball_gencpp)
target_link_libraries(save_kinect_video ${catkin_LIBRARIES})

add_executable(show_camera
  src/camera/show_camera.cpp)
add_dependencies(show_camera unball_gencpp)
target_link_libraries(show_camera ${catkin_LIBRARIES})

add_executable(show_kinect_camera
  src/camera/show_kinect_camera.cpp)
add_dependencies(show_kinect_camera unball_gencpp)
target_link_libraries(show_kinect_camera ${catkin_LIBRARIES})
