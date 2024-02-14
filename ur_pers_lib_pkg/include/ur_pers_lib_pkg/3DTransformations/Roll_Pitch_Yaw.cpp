#include "Roll_Pitch_Yaw.hpp"

using namespace Eigen; 
namespace RPY{

RPY_Angle_s::RPY_Angle_s()
:roll(0)
,pitch(0)
,yaw(0)
{}

RPY_Angle_s::RPY_Angle_s(double roll, double pitch, double yaw)
:roll(roll)
,pitch(pitch)
,yaw(yaw)
{}

void RPY_Angle_s::operator=(const RPY_Angle_s& e_angles)
{
  roll  = e_angles.roll;  
  pitch = e_angles.pitch;  
  yaw   = e_angles.yaw;
}

RPY_Angle_s::~RPY_Angle_s(){}

VectorXd RPY_Angle_s::getVector(){
    VectorXd vec(3);
    vec(0) = this->roll;
    vec(1) = this->pitch;
    vec(2) = this->yaw;
    return vec;
}

Matrix3d RPY_Angle_s::getRotationMatrix(){
  return RPY_To_RotationMatrix(this->roll, this->pitch, this->yaw);
}

VectorXd rotationMatrix_To_RPY(Matrix3d R){
    Eigen::VectorXd vec(3);
    vec(0) = atan2(R(2,1), R(2,2));
    vec(1) = atan2(-R(2,0), sqrt(pow(R(2,1),2) + pow(R(2,2),2)));
    vec(2) = atan2(R(1,0), R(0,0));
    return vec;
}


Matrix3d RPY_To_RotationMatrix(double psi, double theta, double phi)
{
    Matrix3d R;
    
    R(0,0) = cos(phi)*cos(theta);
    R(0,1) = cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi);
    R(0,2) = cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi);

    R(1,0) = sin(phi)*cos(theta);
    R(1,1) = sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi);
    R(1,2) = sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi);

    R(2,0) = -sin(theta);
    R(2,1) = cos(theta)*sin(psi);
    R(2,2) = cos(theta)*cos(psi);
    
    return R;
}

}