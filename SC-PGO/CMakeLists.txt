cmake_minimum_required(VERSION 3.5)
project(aloam_velodyne)

set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
# find_package(rosbag REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)

#find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)

find_package(OpenMP REQUIRED)
find_package(GTSAM REQUIRED QUIET)
list(APPEND CMAKE_MODULE_PATH "/usr/share/cmake/geographiclib")
find_package(GeographicLib REQUIRED)

if(TARGET GeographicLib::GeographicLib)
  set(GEOGRAPHICLIB_TARGET GeographicLib::GeographicLib)
else()
  set(GEOGRAPHICLIB_TARGET ${GeographicLib_LIBRARIES})
endif()

include_directories(
  include
	${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  ${GeographicLib_INCLUDE_DIRS}
)

# catkin_package(
#   CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
#   DEPENDS EIGEN3 PCL 
#   INCLUDE_DIRS include
# )

add_executable(ascanRegistration src/scanRegistration.cpp)
ament_target_dependencies(ascanRegistration rclcpp rclpy std_msgs geometry_msgs nav_msgs sensor_msgs tf2 tf2_ros tf2_geometry_msgs pcl_conversions cv_bridge)
target_link_libraries(ascanRegistration ${PCL_LIBRARIES})

add_executable(alaserOdometry src/laserOdometry.cpp)
ament_target_dependencies(alaserOdometry rclcpp rclpy std_msgs geometry_msgs nav_msgs sensor_msgs tf2 tf2_ros tf2_geometry_msgs pcl_conversions cv_bridge)
target_link_libraries(alaserOdometry ${PCL_LIBRARIES} ${CERES_LIBRARIES})

add_executable(alaserMapping src/laserMapping.cpp)
ament_target_dependencies(alaserMapping rclcpp rclpy std_msgs geometry_msgs nav_msgs sensor_msgs tf2 tf2_ros tf2_geometry_msgs pcl_conversions cv_bridge)
target_link_libraries(alaserMapping ${PCL_LIBRARIES} ${CERES_LIBRARIES})

add_executable(alaserPGO 
  src/laserPosegraphOptimization.cpp
  include/scancontext/Scancontext.cpp
)
ament_target_dependencies(alaserPGO rclcpp rclpy std_msgs geometry_msgs nav_msgs sensor_msgs tf2 tf2_ros tf2_geometry_msgs)
ament_target_dependencies(alaserPGO px4_msgs std_srvs pcl_conversions cv_bridge)
target_compile_options(alaserPGO 
  PRIVATE ${OpenMP_CXX_FLAGS}
)
target_link_libraries(alaserPGO 
  ${PCL_LIBRARIES} ${CERES_LIBRARIES}
  ${OpenMP_CXX_FLAGS}
  gtsam
  ${GEOGRAPHICLIB_TARGET}
)

install(TARGETS
  ascanRegistration
  alaserOdometry
  alaserMapping
  alaserPGO
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()

# add_executable(kittiHelper src/kittiHelper.cpp)
# ament_target_dependencies(kittiHelper rclcpp rclpy std_msgs geometry_msgs nav_msgs sensor_msgs)
# target_link_libraries(kittiHelper ${PCL_LIBRARIES} ${OpenCV_LIBS})
