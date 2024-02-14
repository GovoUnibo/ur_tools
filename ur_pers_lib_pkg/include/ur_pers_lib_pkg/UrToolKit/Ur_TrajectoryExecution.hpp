#pragma once

#include <eigen3/Eigen/Dense>
#include <vector>
#include <iostream>
#include <string>
#include "Ur_JointGroupController.hpp"
#include <ur_pers_lib_pkg/Trajectories/Trajectory3D.hpp>
#include <geometry_msgs/Pose.h>


namespace Universal_Robot{
    
    class Ur_TrajectoryExecution : private Ur_JointGroupController{
        public:
     
            Ur_TrajectoryExecution(ros::NodeHandle& nh, Universal_Robot::UR_TYPE ur_type, std::string prefix, ros::Rate& rate);

            ~Ur_TrajectoryExecution();

            using Ur_JointGroupController::MoveJoint;
            using Ur_JointGroupController::MovePtP;
            using Ur_JointGroupController::getForwardKinematics;
            using Ur_JointGroupController::getJointPositions;
            void setWorld_To_Baselink(double x, double y, double z, double roll, double pitch, double yaw);
            void setEndEff_To_ToolCenterPoint(double x, double y, double z, double roll, double pitch, double yaw);
            
            void moveTrapezoidal(Eigen::VectorXd end_pose, double acc_time, double dec_time, double traj_period, double discretizzation);
            void movePoly5( Eigen::VectorXd end_pose, double T, double discretizzation);

            

        

        private:
            Traj::Trajectory3D trajectory3D;
            std::vector<Eigen::VectorXd> trajectory;
            ros::Publisher traj_pub_visu_rqt;
            void pubTrajectory();
            ros::Rate& rate;
    };
}
