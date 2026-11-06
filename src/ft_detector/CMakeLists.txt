cmake_minimum_required(VERSION 3.0.2)
project(ft_detector)
add_compile_options(-std=c++14 -Wall -Wextra)
find_package(catkin REQUIRED COMPONENTS
  roscpp sensor_msgs geometry_msgs cv_bridge image_transport
  dynamic_reconfigure ft_msgs tf2 tf2_ros)
find_package(OpenCV REQUIRED)
generate_dynamic_reconfigure_options(cfg/Detector.cfg)
catkin_package(CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs cv_bridge image_transport dynamic_reconfigure ft_msgs tf2 tf2_ros)
include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
add_executable(ft_detector_node src/ft_detector_node.cpp)
add_dependencies(ft_detector_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ft_detector_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
