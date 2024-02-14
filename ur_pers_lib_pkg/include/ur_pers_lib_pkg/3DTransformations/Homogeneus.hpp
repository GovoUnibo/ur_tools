#pragma once
#include <math.h>
#include <stdio.h>
#include <iostream>
#include <eigen3/Eigen/Dense>

namespace HomogeneousMatrix
{
    Eigen::Matrix4d Vector_To_HomogeneousMatrix(double x, double y, double z, double psi, double theta, double phi);
    Eigen::VectorXd HomogeneousMatrix_To_Vector(Eigen::Matrix4d T);
}


