cmake_minimum_required(VERSION 3.0.2)
project(ur_pers_lib_pkg)
## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs

  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  pluginlib
  geometric_shapes
  moveit_visual_tools
  

)

find_package(Eigen3 REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    rospy

    
    moveit_core
    moveit_ros_planning_interface
    
    
geometry_msgs
    trajectory_msgs

    
)


include_directories(
  include
  include/ur_pers_lib_pkg
  ${catkin_INCLUDE_DIRS}
)
include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
          include/ur_pers_lib_pkg/3DTransformations/Roll_Pitch_Yaw.cpp 
          include/ur_pers_lib_pkg/3DTransformations/AngularVelocity.cpp
          include/ur_pers_lib_pkg/3DTransformations/Homogeneus.cpp
          include/ur_pers_lib_pkg/3DTransformations/Quaternion/Quaternion_s.cpp           
          include/ur_pers_lib_pkg/3DTransformations/Quaternion/Quaternion_Operations.cpp
          include/ur_pers_lib_pkg/3DTransformations/MatrixVectorRelations.cpp

          include/ur_pers_lib_pkg/UrToolKit/Ur_Utils/Ur_DenavitHartenberg.cpp
          include/ur_pers_lib_pkg/UrToolKit/Ur_Utils/Ur_JointNames.cpp
          include/ur_pers_lib_pkg/UrToolKit/Ur_Utils/UR_Types.hpp
          include/ur_pers_lib_pkg/UrToolKit/Ur_kins/Kinematics.cpp
          include/ur_pers_lib_pkg/UrToolKit/Ur_kins/FindNearestConfiguration.cpp
          include/ur_pers_lib_pkg/UrToolKit/Ur_JointGroupController.cpp
          include/ur_pers_lib_pkg/UrToolKit/Ur_TrajectoryExecution.cpp
          include/ur_pers_lib_pkg/UrToolKit/Ur_ComplianceControl.cpp


          include/ur_pers_lib_pkg/Trajectories/Poly5Trajectory.cpp
          include/ur_pers_lib_pkg/Trajectories/TrapezoidalTrajectory.cpp
          include/ur_pers_lib_pkg/Trajectories/ScaledTrajectory.cpp
          include/ur_pers_lib_pkg/Trajectories/Trajectory3D.cpp

          include/ur_pers_lib_pkg/Ros_Controllers/JointGroupPub.cpp
          include/ur_pers_lib_pkg/Ros_Controllers/JS_Subscriber.cpp
          include/ur_pers_lib_pkg/Ros_Controllers/JointGroupPubSub.cpp
          
          include/ur_pers_lib_pkg/Controllers/ComplianceControl.cpp
          include/ur_pers_lib_pkg/Controllers/LQRController.cpp

          include/ur_pers_lib_pkg/ForceSensor/MassTorqueEstimation/EeRots_for_MassEstimation.cpp
          include/ur_pers_lib_pkg/ForceSensor/MassTorqueEstimation/PayLoad_Id_LeastSquare.cpp
          include/ur_pers_lib_pkg/ForceSensor/FT_Reader/SensorReader.cpp

          include/ur_pers_lib_pkg/Filters/Filters.cpp
    )

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# install(
#   TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )
# install(
#   DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )
      
# add_executable(grav_comp src/gripper_estimation.cpp)
                  
# target_link_libraries(grav_comp
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
#   ${Eigen3_LIBRARIES}
#   ${PROJECT_NAME}
# )
add_executable(split_joint_state src/split_joint_state.cpp)
                  
target_link_libraries(split_joint_state
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${Eigen3_LIBRARIES}
  ${PROJECT_NAME}
)
# add_executable(jac_moveit src/get_moveit_jacobian.cpp
#               )

# target_link_libraries(jac_moveit
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
#   ${Eigen3_LIBRARIES}
# )

