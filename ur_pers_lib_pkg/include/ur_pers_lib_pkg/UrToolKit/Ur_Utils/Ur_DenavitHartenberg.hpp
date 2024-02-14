#pragma once

#include <eigen3/Eigen/Dense>
#include "UR_Types.hpp"
#include <iostream>

namespace Universal_Robot
{
    struct DH_Parameters{
        DH_Parameters(UR_TYPE robot_type);
        // distruttore
        ~DH_Parameters();
        double d1;
        double a2;
        double a3;
        double d4;
        double d5;
        double d6;

        Eigen::VectorXd a_coef;
        Eigen::VectorXd d_coef;
        Eigen::VectorXd alpha_coef;

        Eigen::Matrix4d _DH_MatrixTF(double a, double alpha, double d, double theta);

    };
} // namespace Universal_Robot