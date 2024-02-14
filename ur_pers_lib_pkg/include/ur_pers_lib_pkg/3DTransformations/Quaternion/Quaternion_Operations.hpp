#pragma once

#include <eigen3/Eigen/Dense>
#include "Quaternion_s.hpp"





RPY::RPY_Angle_s Quaternion_To_RPY(const MyQuaternion::Quaternion_s q);
MyQuaternion::Quaternion_s RPY_To_Quaternion(double roll, double pitch, double yaw);
MyQuaternion::Quaternion_s quaternion_negation(const MyQuaternion::Quaternion_s v);
MyQuaternion::Quaternion_s quaternion_conjugate(const MyQuaternion::Quaternion_s q);
MyQuaternion::Quaternion_s quaternion_scalar_product(MyQuaternion::Quaternion_s v, double t);
MyQuaternion::Quaternion_s quaternion_product(MyQuaternion::Quaternion_s q1, MyQuaternion::Quaternion_s q2);
MyQuaternion::Quaternion_s quaternion_minus(const MyQuaternion::Quaternion_s v1, const MyQuaternion::Quaternion_s v0);
MyQuaternion::Quaternion_s quaternion_plus(const MyQuaternion::Quaternion_s v1, const MyQuaternion::Quaternion_s v0);
double quaternion_norm(const MyQuaternion::Quaternion_s q);
MyQuaternion::Quaternion_s quaternion_normalize(MyQuaternion::Quaternion_s q);
MyQuaternion::Quaternion_s rotation_composition(const MyQuaternion::Quaternion_s q_01, const MyQuaternion::Quaternion_s q_02);
double quaternion_theta(const MyQuaternion::Quaternion_s q);
MyQuaternion::Quaternion_s rotMatrix_toQuaternion(const Eigen::Matrix3d & rotMatrix);
double dot_product(MyQuaternion::Quaternion_s q_0 , MyQuaternion::Quaternion_s q_1);
MyQuaternion::Quaternion_s slerp(MyQuaternion::Quaternion_s quat_a, MyQuaternion::Quaternion_s quat_b, double t);
AngVel::AngularVelocity_struct rotationVelocity(MyQuaternion::Quaternion_s quat_actual, MyQuaternion::Quaternion_s quat_obj, double T_period);
