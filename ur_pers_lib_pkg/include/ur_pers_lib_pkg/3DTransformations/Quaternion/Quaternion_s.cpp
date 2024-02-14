#include "Quaternion_Operations.hpp"


using namespace Eigen;
using namespace AngVel;
using namespace RPY;

namespace MyQuaternion{

Quaternion_s::Quaternion_s()
:w(0)
,x(0)
,y(0)
,z(0)
{}

Quaternion_s::Quaternion_s(double w, double x, double y, double z)
:w(w)
,x(x)
,y(y)
,z(z)
{}

Quaternion_s::Quaternion_s(double roll, double pitch, double yaw) 
{
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    Quaternion_s q;
    w = cr * cp * cy + sr * sp * sy;
    x = sr * cp * cy - cr * sp * sy;
    y = cr * sp * cy + sr * cp * sy;
    z = cr * cp * sy - sr * sp * cy;
}

Quaternion_s::Quaternion_s(const Quaternion_s& source)
:w(source.w)
,x(source.x)
,y(source.y)
,z(source.z)
{}

Quaternion_s::Quaternion_s(const RPY::RPY_Angle_s& rpy){
    Quaternion_s q = RPY_To_Quaternion(rpy.roll, rpy.pitch, rpy.yaw);
    this->w=q.w; 
    this->x=q.x; 
    this->y=q.y; 
    this->z=q.z;
}


Quaternion_s::Quaternion_s(const Eigen::Matrix3d& m){
  Quaterniond q(m);
  q.normalize();
  this->w=q.w();
  this->x=q.x();
  this->y=q.y();
  this->z=q.z();
}


Quaternion_s::~Quaternion_s(){}

RPY_Angle_s Quaternion_s::getRPY()
{
    RPY_Angle_s angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (w * x + y * z);
    double cosr_cosp = 1 - 2 * (x * x + y * y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (w * y - z * x);
    if (std::abs(sinp) >= 1)
        angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}

Quaternion_s Quaternion_s::operator*=(const Quaternion_s& q){
    Quaternion_s result;

    result.w = (w*q.w) - (x*q.x) - (y*q.y) - (z*q.z);
    result.x = (w*q.x) + (x*q.w) + (y*q.z) - (z*q.y);
    result.y = (w*q.y) + (y*q.w) + (z*q.x) - (x*q.z);
    result.z = (w*q.z) + (z*q.w) + (x*q.y) - (y*q.x);

    return result;
}



Quaternion_s Quaternion_s::operator+=(const Quaternion_s& q)
{ 
    Quaternion_s result(w+q.w, x+q.x, y+q.y, z+q.z);

    return result;
}

void Quaternion_s::operator=(const Quaternion_s& q) //esempio q_a = q_b
{ 
      this->w=q.w; 
      this->x=q.x; 
      this->y=q.y; 
      this->z=q.z;
}

void Quaternion_s::operator=(const RPY::RPY_Angle_s& e_angles){
    Quaternion_s q = RPY_To_Quaternion(e_angles.roll, e_angles.pitch, e_angles.yaw);
    this->w=q.w; 
    this->x=q.x; 
    this->y=q.y; 
    this->z=q.z;
}

void Quaternion_s::operator=(const Eigen::Matrix3d& v){
  Quaterniond q(v);
  q.normalize();
  this->w=q.w();
  this->x=q.x();
  this->y=q.y();
  this->z=q.z();
}

Quaternion_s Quaternion_s::conjugate() {  
  this->w =  this->w;
  this->x = -this->x;
  this->y = -this->y;
  this->z = -this->z;

  return *this;
}

Quaternion_s Quaternion_s::negation() {
  this->w = -this->w;
  this->x = -this->x;
  this->y = -this->y;
  this->z = -this->z;

  return *this;
}

Quaternion_s Quaternion_s::scalar_product(double t) {

  this->w = t * this->w;
  this->x = t * this->x;
  this->y = t * this->y;
  this->z = t * this->z;

  return *this;
}

double Quaternion_s::getNorm() const
{
    return sqrt( pow(w, 2.0) + pow(x, 2.0) + pow(y, 2.0) + pow(z, 2.0) );
}

Quaternion_s Quaternion_s::normalize() 
{
  double norm = getNorm();

  this->w = this->w/norm;
  this->x = this->x/norm;
  this->y = this->y/norm;
  this->z = this->z/norm;

  return *this;

}

double Quaternion_s::dot_product(const Quaternion_s& q) 
{

    return this->w*q.w + this->x*q.x + this->y*q.y + this->z*q.z;
}

Quaternion_s Quaternion_s::slerp(Quaternion_s quat_target, double t){
        this->normalize();
        quat_target.normalize();

        double dot = this->dot_product(quat_target); //cosHalfTheta


        if (dot < 0.0f) {    // If the dot product is negative, slerp won't take the shorter path. Note that v1 and -v1 are equivalent when
            // the negation is applied to all four components. Fix by reversing one Quaternion_s.
            quat_target = quat_target.negation();
            dot = -dot;
        }
        
        const double DOT_THRESHOLD = 0.9995;         // If the inputs are too close for comfort, linearly interpolate and normalize the result.
        if (dot > DOT_THRESHOLD) {
            Quaternion_s result = *this + t*(quat_target - *this);
            result.normalize();
            return result;
        }

        // Since dot is in range [0, DOT_THRESHOLD], acos is safe
        double theta_0      = acos(dot);        // theta_0 = angle between input vectors
        double theta        = theta_0*t;          // theta = angle between v0 and result
        double sin_theta    = sin(theta);     // compute this value only once
        double sin_theta_0  = sin(theta_0); // compute this value only once

        double s0 = cos(theta) - dot * sin_theta / sin_theta_0;  // == sin(theta_0 - theta) / sin(theta_0)
        double s1 = sin_theta / sin_theta_0;

        return (s0 * *this) + (s1 * quat_target);
}






/*
########  ######  #     #   ######     ######
#           ##    ##    #   #         #
#           ##    # #   #   #         #                                                                                                                  
#######     ##    #  #  #   ######     #####                                                                                                                        
#           ##    #   # #   #               #                                                                                                               
#           ##    #    ##   #               #
#         ######  #     #   ######    ######    ##########
*/





Quaternion_s operator+(const Quaternion_s& lho, const Quaternion_s& rho)
{    
     Quaternion_s result(lho.w + rho.w, lho.x + rho.x, lho.y + rho.y, lho.z + rho.z);
    return result;
}

Quaternion_s operator*(const Quaternion_s& q1, const Quaternion_s& q2) 
{
  Quaternion_s res;

  res.w = (q1.w*q2.w) - (q1.x*q2.x) - (q1.y*q2.y) - (q1.z*q2.z);
  res.x = (q1.w*q2.x) + (q1.x*q2.w) + (q1.y*q2.z) - (q1.z*q2.y);
  res.y = (q1.w*q2.y) + (q1.y*q2.w) + (q1.z*q2.x) - (q1.x*q2.z);
  res.z = (q1.w*q2.z) + (q1.z*q2.w) + (q1.x*q2.y) - (q1.y*q2.x);

  return res;
}

Quaternion_s operator*(const double k, const Quaternion_s& q) 
{
  Quaternion_s res;

  res.w = k * q.w;
  res.x = k * q.x;
  res.y = k * q.y;
  res.z = k * q.z;

  return res;
}

Quaternion_s operator-(const Quaternion_s& v1, const Quaternion_s& v0)
{

  Quaternion_s vdiff;
  vdiff.w = v1.w - v0.w;
  vdiff.x = v1.x - v0.x;
  vdiff.y = v1.y - v0.y;
  vdiff.z = v1.z - v0.z;

  return vdiff;

}



} // namespace QuatS