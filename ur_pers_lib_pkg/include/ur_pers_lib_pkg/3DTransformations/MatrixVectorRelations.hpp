#pragma once

#include <math.h>
#include <stdio.h>
#include <iostream>
#include <eigen3/Eigen/Dense>


bool areRotMatricesComplanar(const Eigen::Matrix3d& A, const Eigen::Matrix3d& B);
bool areVectorsLinearlyDependent(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2);

bool areVectorsComplanar(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2);