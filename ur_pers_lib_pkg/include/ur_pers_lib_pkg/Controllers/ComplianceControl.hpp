#ifndef ADMITTANCECONTROLLER_H
#define ADMITTANCECONTROLLER_H

#include "ros/ros.h"

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "eigen3/Eigen/Dense"

#include <fstream>
#include <cstring>
#include <iostream>

#include <exception>
#include <string>


namespace Controller{
    class Admittance
    {
    public:
        Admittance(ros::Rate rate_);

        // Eigen::VectorXd computeAdmittance(std::vector<double> force, std::vector<double> torque, std::vector<double> reference_pose);
        void fixRobotSetPoint(Eigen::VectorXd robot_setPoint); // robot setpoint da cui il robot deve sentire una forza per spostarsi
        Eigen::VectorXd computeAdmittance(Eigen::VectorXd F_ext );
        
    private:

        Eigen::MatrixXd M_d = Eigen::MatrixXd::Zero(6,6);
        Eigen::MatrixXd D_d = Eigen::MatrixXd::Zero(6,6);
        Eigen::MatrixXd K_d = Eigen::MatrixXd::Zero(6,6);

        ros::Rate loop_rate_;

        Eigen::VectorXd robot_setPoint = Eigen::VectorXd::Zero(6); // reference position
        Eigen::VectorXd delta_acc = Eigen::VectorXd::Zero(6);
        Eigen::VectorXd delta_vel = Eigen::VectorXd::Zero(6);
        Eigen::VectorXd delta_pos = Eigen::VectorXd::Zero(6);

        ros::Duration dt;

        int debug = 0;

    };
} // namespace Controller

#endif // ADMITTANCECONTROLLER_H