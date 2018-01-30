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
  OpenCV REQUIRED
)

add_definitions(-std=c++11)

find_package(Boost REQUIRED COMPONENTS system)


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
  src/vision/algorithm.cpp
  src/vision/algorithm_factory.cpp
  src/vision/ball_identification.cpp
  src/vision/ball_in_range_color_segmentation.cpp
  src/vision/config_parser.cpp
  src/vision/dummy_identification_algorithm.cpp
  src/vision/dummy_segmentation_algorithm.cpp
  src/vision/find_robots_by_color.cpp
  src/vision/hsv_in_range_color_segmentation.cpp
  src/vision/robot_identification.cpp
  src/vision/new_robot_identification.cpp
  src/vision/identification_algorithm.cpp
  src/vision/identifier.cpp
  src/vision/raw_image.cpp
  src/vision/segmentation_algorithm.cpp
  src/vision/segmenter.cpp
  src/vision/tracked_object.cpp
  src/vision/tracker.cpp
  src/vision/vision.cpp
  src/vision/vision_gui.cpp
 )
add_dependencies(vision_node unball_gencpp)
target_link_libraries(vision_node ${catkin_LIBRARIES})

add_executable(camera
  src/camera/camera.cpp)
add_dependencies(camera unball_gencpp)
target_link_libraries(camera ${catkin_LIBRARIES})


add_executable(test_calibration
  src/test_calibration/test_calibration_node.cpp)
add_dependencies(test_calibration unball_gencpp)
target_link_libraries(test_calibration ${catkin_LIBRARIES})

add_executable(dummy_camera
  src/dummy_camera/dummy_camera.cpp)
add_dependencies(dummy_camera unball_gencpp)
target_link_libraries(dummy_camera ${catkin_LIBRARIES})

add_executable(dummy_image
  src/dummy_camera/dummy_image.cpp)
add_dependencies(dummy_image unball_gencpp)
target_link_libraries(dummy_image ${catkin_LIBRARIES})

add_executable(calibration_node
  src/calibration/calibration_node.cpp
  src/calibration/capture_points.cpp
  src/calibration/file_manager.cpp
  src/calibration/matching.cpp
  src/calibration/select_field.cpp
  src/calibration/color_calibration.cpp)
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


add_executable(pixel_to_metric_conversion
  src/pixel_to_metric_conversion/pixel_to_metric_conversion_node.cpp)
add_dependencies(pixel_to_metric_conversion unball_gencpp)
target_link_libraries(pixel_to_metric_conversion ${catkin_LIBRARIES})


