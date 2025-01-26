cmake_minimum_required(VERSION 2.8)
project(orb_slam3_ros_wrapper)

# Change this to your installation of ORB-SLAM3. Default is ~/
set(ORB_SLAM3_DIR
   /ORB_SLAM3
)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package (catkin REQUIRED COMPONENTS
   roscpp
   rospy
   rosbag
   std_msgs
   cv_bridge
   image_transport
   tf
   sensor_msgs
   message_generation
)

find_package(OpenCV REQUIRED)
MESSAGE("OPENCV VERSION:")
MESSAGE(${OpenCV_VERSION})

find_package(Eigen3 REQUIRED)
MESSAGE("Eigen3 VERSION:")
MESSAGE(${Eigen3_VERSION})

find_package(Pangolin REQUIRED)

catkin_package (
   CATKIN_DEPENDS roscpp rospy std_msgs cv_bridge image_transport tf sensor_msgs message_runtime
   LIBRARIES {PROJECT_NAME}
)

include_directories(
   ${ORB_SLAM3_DIR}
   ${ORB_SLAM3_DIR}/include
   ${ORB_SLAM3_DIR}/include/CameraModels
   ${ORB_SLAM3_DIR}/Thirdparty/Sophus
   ${PROJECT_SOURCE_DIR}/include
   ${EIGEN3_INCLUDE_DIR}
   ${catkin_INCLUDE_DIRS}
   ${Pangolin_INCLUDE_DIRS}
)

# Build ROS nodes
set(LIBS
   ${ORB_SLAM3_DIR}/lib/libORB_SLAM3.so
   ${EIGEN3_LIBS}
   ${catkin_LIBRARIES}
   ${Pangolin_LIBRARIES}
   ${OpenCV_LIBS}
)

# mono node
add_executable (${PROJECT_NAME}_mono
   src/mono_node.cc
   src/common.cc
)
target_link_libraries(${PROJECT_NAME}_mono
   ${LIBS}
)


# stereo node
add_executable (${PROJECT_NAME}_stereo
   src/stereo_node.cc
   src/common.cc
)
target_link_libraries(${PROJECT_NAME}_stereo
   ${LIBS}
)

# stereo node (replay)
add_executable (${PROJECT_NAME}_stereo_replay
   src/replay_stereo_node.cc
   src/common.cc
)
target_link_libraries(${PROJECT_NAME}_stereo_replay
   ${LIBS}
)


# mono-inertial node
add_executable (${PROJECT_NAME}_mono_inertial
   src/mono_inertial_node.cc
   src/common.cc
)
target_link_libraries(${PROJECT_NAME}_mono_inertial
   ${LIBS}
)


# stereo-inertial node
add_executable (${PROJECT_NAME}_stereo_inertial
   src/stereo_inertial_node.cc
   src/common.cc
)
target_link_libraries(${PROJECT_NAME}_stereo_inertial
   ${LIBS}
)


# RGB-D node
add_executable (${PROJECT_NAME}_rgbd
   src/rgbd_node.cc
   src/common.cc
)
target_link_libraries(${PROJECT_NAME}_rgbd
   ${LIBS}
)
