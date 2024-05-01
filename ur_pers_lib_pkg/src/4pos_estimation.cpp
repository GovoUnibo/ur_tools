#include <ur_pers_lib_pkg/ForceSensor/MassTorqueEstimation/PayLoad_Id_LeastSquare.hpp>
#include <ur_pers_lib_pkg/UrToolKit/Ur_TrajectoryExecution.hpp>

#include <cmath>
using namespace std;
using namespace Universal_Robot;
using namespace Eigen;
using namespace RPY;

int main(int argc, char** argv) {

    ros::init(argc, argv, "test");
    ros::NodeHandle nh;
    float publish_rate = 500;
    ros::Rate rate(publish_rate);
    ros::Rate sleep_rate(1);

    string robot_name = "left"; 
    
    Ur_TrajectoryExecution left_arm(nh, UR_TYPE::UR5e, robot_name, rate);
    
    PayLoad_LeastSquare payload(nh, rate, "/ft_sensor");
    
    // standard values
    double t_acc = 0.5, t_dec = 0.5, t_traj = 2.0; // creating a trapezio isoscele
    // variabili d'appoggio
    VectorXd pos1(6), pos2(6), pos3(6), pos4(6);

    // coputing bias using mass
    // payload.computeBiasUsingGripperMass(1.48);
    
    // position 1
    pos1 << -1.56386, -0.76384, -1.76386, 3.91304, 1.56386, -5.34765e-06;
    left_arm.moveJointTrapezoidal(pos1, t_acc, t_dec, t_traj);
    ros::Duration(1).sleep();
    payload.registerFTValues(pos1.tail(3));
    cout << endl;

    // position 2
    pos2 << -1.56386, -0.76384, -1.76386, 5.7, 0.0, 0.0;
    left_arm.moveJointTrapezoidal(pos2, t_acc, t_dec, t_traj);
    ros::Duration(1).sleep();
    payload.registerFTValues(pos2.tail(3));
    cout << endl;
    
    // position 3
    pos3 << -1.56386, -0.76384, -1.76386, 3.4, 0.0, -5.34765e-06;
    left_arm.moveJointTrapezoidal(pos3, t_acc, t_dec, t_traj);
    ros::Duration(3).sleep();
    payload.registerFTValues(left_arm.getForwardKinematics().tail(3));
    cout << endl;

    // position 4
    pos4 << -1.56386, -0.76384, -1.76386, 7.2, 1.3, -5.34765e-06;
    pos4(5) = 0.0;
    left_arm.moveJointTrapezoidal(pos4, t_acc, t_dec, t_traj);
    ros::Duration(3).sleep();
    payload.registerFTValues(left_arm.getForwardKinematics().tail(3));
    cout << endl;
    
    payload.CoG_Identification();
    

    while (ros::ok()) {

        
        VectorXd V_fwrd = left_arm.getForwardKinematics();
        cout << "qui1" << endl;
        cout << payload.getFTVector(V_fwrd(3), V_fwrd(4), V_fwrd(5), false).transpose() << endl;
        cout << "qui2" << endl;
        cout << payload.getFTVector(V_fwrd(3), V_fwrd(4), V_fwrd(5), true).transpose() << endl;
        cout << "qui3" << endl;
        // cout << "Force in Base frame: " << endl << ForceTorque.head(3) << endl;

        sleep_rate.sleep();
    }


    return 0;
}