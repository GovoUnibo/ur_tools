#pragma once
#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <ur_pers_lib_pkg/3DTransformations/Roll_Pitch_Yaw.hpp>
#include <ur_pers_lib_pkg/3DTransformations/AngularVelocity.hpp>


namespace MyQuaternion{


    struct Quaternion_s {
        double w, x, y, z;

        Quaternion_s();
        Quaternion_s(double w, double x, double y, double z);
        Quaternion_s(double roll, double pitch, double yaw);
        Quaternion_s(const Quaternion_s& source);
        Quaternion_s(const Eigen::Matrix3d& m);
        Quaternion_s(const RPY::RPY_Angle_s& rpy);
        ~Quaternion_s();
        Quaternion_s operator*=(const Quaternion_s& q);
        Quaternion_s operator+=(const Quaternion_s& q);
        void operator=(const Quaternion_s& q);
        void operator=(const RPY::RPY_Angle_s& e_angles);
        void operator=(const Eigen::Matrix3d& m);
        double getNorm() const;
        double dot_product(const Quaternion_s& q);
        Quaternion_s normalize();
        RPY::RPY_Angle_s getRPY();
        Quaternion_s conjugate();
        Quaternion_s negation();
        Quaternion_s scalar_product(double t);

        Quaternion_s slerp(Quaternion_s quat_target, double t);
    };

    Quaternion_s operator+(const Quaternion_s& lho, const Quaternion_s& rho);
    Quaternion_s operator*(const Quaternion_s& q1, const Quaternion_s& q2);
    Quaternion_s operator*(const double k, const Quaternion_s& q);
    Quaternion_s operator-(const Quaternion_s& v1, const Quaternion_s& v0);

} // namespace MyQuaternion