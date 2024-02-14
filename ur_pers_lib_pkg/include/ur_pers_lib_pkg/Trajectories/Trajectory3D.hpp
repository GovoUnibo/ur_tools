#pragma once

#include <math.h>
#include <vector>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <ur_pers_lib_pkg/Trajectories/ScaledTrajectory.hpp>
#include <ur_pers_lib_pkg/3DTransformations/Quaternion/Quaternion_Operations.hpp>

namespace Traj{

    class Trajectory3D : private ScaledTrajectory
    {
    private:
        
        double T, time_discretizzation;
        void resize_pos_vectors(int num_of_points);
        void resize_vel_vectors(int num_of_points);
        void resize_acc_vectors(int num_of_points);
        void resize_rot_vectors(int num_of_points);
        std::vector<double> position_x, position_y, position_z;
        std::vector<double> velocity_x, velocity_y, velocity_z;
        std::vector<double> acceleration_x, acceleration_y, acceleration_z;

        
        // rotation variables for slerp trajectory
        std::vector<double> theta_x, theta_y, theta_z;
        std::vector<double> omega_x, omega_y, omega_z;
        MyQuaternion::Quaternion_s q0, qf;
        void rotTraj(); // compute the rotation and angular velocity trajectory given a scaled trajectory for s and the initial and final rotation
        

    public:
        Trajectory3D();
        Trajectory3D(TrajParam traj_param, std::string traj_type);
        ~Trajectory3D();

        using ScaledTrajectory::setTrajectoryType;
        //position methods
        std::vector<Eigen::Vector3d> get3DTrajPositionPoints(Eigen::Vector3d start_pose, Eigen::Vector3d end_pose);
        std::vector<Eigen::Vector3d> get3DTrajVelocityPoints(Eigen::Vector3d start_pose, Eigen::Vector3d end_pose);
        std::vector<Eigen::Vector3d> get3DTrajAccelerationPoints(Eigen::Vector3d start_pose, Eigen::Vector3d end_pose);

        std::vector<Eigen::Vector3d> get3DRotPositionPoints(Eigen::Vector3d start_rot, Eigen::Vector3d end_rot);
        std::vector<Eigen::Vector3d> get3DTrajAngularVelocityPoints(Eigen::Vector3d start_rot, Eigen::Vector3d end_rot);

        std::vector<Eigen::VectorXd> getPosRotTrajPoints(Eigen::VectorXd start_pose, Eigen::VectorXd end_pose);

    };

} // namespace Traj3D


