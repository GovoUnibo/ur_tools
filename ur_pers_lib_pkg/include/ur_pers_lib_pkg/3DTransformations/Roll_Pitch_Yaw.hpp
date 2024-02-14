#pragma once

#include <math.h>
#include <stdio.h>
#include <iostream>
#include <eigen3/Eigen/Dense>


namespace RPY{
struct RPY_Angle_s {
    double roll, pitch, yaw;

    RPY_Angle_s();
    RPY_Angle_s(double roll, double pitch, double yaw);
    void operator=(const RPY_Angle_s& e_angles);
    Eigen::VectorXd getVector();
    Eigen::Matrix3d getRotationMatrix();
    ~RPY_Angle_s();
};

Eigen::VectorXd rotationMatrix_To_RPY(Eigen::Matrix3d R);
Eigen::Matrix3d RPY_To_RotationMatrix(double psi, double theta, double phi);


} // namespace RPY