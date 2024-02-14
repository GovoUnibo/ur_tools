#include "Homogeneus.hpp"

using namespace Eigen;

namespace HomogeneousMatrix
{

Matrix4d Vector_To_HomogeneousMatrix(double x, double y, double z, double psi, double theta, double phi)
{
    Matrix4d T;
    
    T(0,0) = cos(phi)*cos(theta);
    T(0,1) = cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi);
    T(0,2) = cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi);
    T(0,3) = x;

    T(1,0) = sin(phi)*cos(theta);
    T(1,1) = sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi);
    T(1,2) = sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi);
    T(1,3) = y;

    T(2,0) = -sin(theta);
    T(2,1) = cos(theta)*sin(psi);
    T(2,2) = cos(theta)*cos(psi);
    T(2,3) = z;

    T(3,0) = 0;
    T(3,1) = 0;
    T(3,2) = 0;
    T(3,3) = 1;
    
    return T;
}



VectorXd HomogeneousMatrix_To_Vector(Matrix4d T){
    Eigen::VectorXd vec(6);
    vec(0) = T(0,3);
    vec(1) = T(1,3);
    vec(2) = T(2,3);
    vec(3) = atan2(T(2,1), T(2,2));
    vec(4) = atan2(-T(2,0), sqrt(pow(T(2,1),2) + pow(T(2,2),2)));
    vec(5) = atan2(T(1,0), T(0,0));
    return vec;
}

}



