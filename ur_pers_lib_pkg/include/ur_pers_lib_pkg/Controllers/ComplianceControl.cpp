#include <ur_pers_lib_pkg/Controllers/ComplianceControl.hpp>



namespace Controller{
Admittance::Admittance(ros::Rate rate_) 
: M_d(6,6)
, D_d(6,6)
, K_d(6,6)
, loop_rate_(rate_)
{
    M_d <<  16, 0, 0, 0, 0, 0,  //2
            0, 16, 0, 0, 0, 0,
            0, 0, 16, 0, 0, 0,
            0, 0, 0, 2, 0, 0,
            0, 0, 0, 0, 2, 0,
            0, 0, 0, 0, 0, 2;

    D_d <<  25, 0, 0, 0, 0, 0,   //10
            0, 25, 0, 0, 0, 0,
            0, 0, 25, 0, 0, 0,
            0, 0, 0, 25, 0, 0,
            0, 0, 0, 0, 25, 0,
            0, 0, 0, 0, 0, 25;

    K_d <<  35, 0, 0, 0, 0, 0, //25
            0, 35, 0, 0, 0, 0,
            0, 0, 35, 0, 0, 0,
            0, 0, 0, 5, 0, 0,
            0, 0, 0, 0, 5, 0,
            0, 0, 0, 0, 0, 5;

    dt = loop_rate_.expectedCycleTime();
    std::cout << "Admittance initialized." << std::endl;

}

void Admittance::fixRobotSetPoint(Eigen::VectorXd robot_setPoint){
    this->robot_setPoint = robot_setPoint;
}



Eigen::VectorXd Admittance::computeAdmittance(Eigen::VectorXd F_ext) { 

    // acceleration due to external force
        delta_acc = M_d.inverse() * (F_ext - D_d * delta_vel - K_d * delta_pos);
    // ROS_INFO_STREAM("delta acc: \n" << delta_acc);

    // velocity due to acceleration
        delta_vel += delta_acc * dt.toSec();
    // ROS_INFO_STREAM("delta vel: \n" << delta_vel);

    // position due to velocity
        delta_pos += delta_vel * dt.toSec();
    // ROS_INFO_STREAM("delta pos: \n" << delta_pos);



    return robot_setPoint + delta_pos;

}


}// namespace Controller