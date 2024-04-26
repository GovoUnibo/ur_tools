#pragma once

#include <ur_pers_lib_pkg/3DTransformations/Homogeneus.hpp>
#include <ur_pers_lib_pkg/3DTransformations/Roll_Pitch_Yaw.hpp>
#include <vector>
#include <eigen3/Eigen/Dense>
// libreria per runtime_error
#include <stdexcept>
// libreria per cout
#include <iostream>
#include "EeRots_for_MassEstimation.hpp"
#include <ur_pers_lib_pkg/ForceSensor/FT_Reader/SensorReader.hpp>

class PayLoad_LeastSquare : private SensorReader
{
    public:
    // constructor
    PayLoad_LeastSquare(ros::NodeHandle& nh, ros::Rate sensor_update_rate_hz, std::string topic_name);
    // destructor
    ~PayLoad_LeastSquare();

    void computeBiasUsingGripperMass(double gripper_mass = 0);
    void computeBiasUsing2RobotPos();
    void setBias(double fx0, double fy0, double fz0, double tx0, double ty0, double tz0);

    void registerFTValues(Eigen::Vector3d eeRot = Eigen::Vector3d(0, 0, 0));

    void CoG_Identification();

    using  SensorReader::setRotMatrix_sb;
    using  SensorReader::getWrenchInBaseFrame;
    using  SensorReader::getWrenchInSensorFrame;

    Eigen::VectorXd getFTVector(double rx, double ry, double rz, bool in_base_frame=true);

private:
    void estimationOfMass(); 
    double Fx0, Fy0, Fz0, Tx0, Ty0, Tz0; // initial force and torque
    double mass;
    std::vector<Eigen::VectorXd> FT_Bias;
    std::vector<Eigen::Vector3d> massEstimationForces;
    std::vector<Eigen::Vector3d> gravityRotatedVector;
    std::vector<Eigen::MatrixXd> Forces;
    std::vector<Eigen::Vector3d> Torques;
    const Eigen::Vector3d gravityVector = Eigen::Vector3d(0, 0, -9.81);
    double cgX, cgY, cgZ; // center of gravity
    double k1, k2, k3; // k1, k2, k3 are the components of the vector k

    Eigen::Vector3d getTorqueEstimationVector(double fx, double fy, double fz); 
    Eigen::VectorXd ftEstimation;

    Eigen::Matrix3d R_sb;
    // Vector3d forceEstimation;
    // Vector3d torqueEstimation;

};

