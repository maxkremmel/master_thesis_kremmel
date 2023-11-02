cmake_minimum_required(VERSION 3.0.2)
project(master_thesis_kremmel)

find_package(PCL REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs 
  std_msgs
  geometry_msgs
  pcl_ros
  pcl_conversions
  message_generation
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  Landmark.srv
  MoveRobot.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

pkg_check_modules(EIGEN3 eigen3 REQUIRED)

catkin_package(
    INCLUDE_DIRS
    CATKIN_DEPENDS roscpp
                   sensor_msgs 
                   std_msgs
                   pcl_ros
                   pcl_conversions
  )

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${VTK_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(TransformLaserScan src/transformLaserScan.cpp)
target_link_libraries(TransformLaserScan ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(LandmarkProcessor src/landmarkProcessing.cpp)
target_link_libraries(LandmarkProcessor ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(EKFSlam src/ekfSlam.cpp)
target_link_libraries(EKFSlam ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} ${PCL_LIBRARIES})

add_executable(Evaluator src/evaluator.cpp)
target_link_libraries(Evaluator ${catkin_LIBRARIES})
add_executable(CmdVelSmoother src/cmdVelSmoother.cpp)
target_link_libraries(CmdVelSmoother ${catkin_LIBRARIES})

#add_executable(PubToCmdVel src/pubToCmdVel.cpp)
#target_link_libraries(PubToCmdVel ${catkin_LIBRARIES})
