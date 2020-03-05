cmake_minimum_required(VERSION 2.8.3)
project(grasp_pipeline)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  pcl_conversions
  pcl_ros
  point_cloud_segmentation
  cv_bridge
  image_transport
  trajectory_msgs
)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
catkin_python_setup()

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   GraspPreshape.srv
   AllegroConfig.srv
   PalmGoalPoseWorld.srv
   # MoveArm.srv
   ManageMoveitScene.srv
   SimGraspData.srv
   RealGraspData.srv
   UpdateInfPreshape.srv
   StraightLinePlan.srv
   SaveVisualData.srv
   GraspDataBagging.srv
   UpdateObjectGazebo.srv
   MoveObjectGazebo.srv
 )


## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
   geometry_msgs
   point_cloud_segmentation
   trajectory_msgs
 )

include_directories(
  ${catkin_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(save_visual_data src/save_visual_data_server.cpp)
add_dependencies(save_visual_data grasp_pipeline_generate_messages_cpp)
target_link_libraries(save_visual_data ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
