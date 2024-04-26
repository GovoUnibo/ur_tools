#include <ur_pers_lib_pkg/ForceSensor/MassTorqueEstimation/PayLoad_Id_LeastSquare.hpp>


#include <ur_pers_lib_pkg/UrToolKit/Ur_TrajectoryExecution.hpp>

#include <cmath>
using namespace std;
using namespace Universal_Robot;

using namespace Eigen;
using namespace RPY;

double norm(VectorXd vec) {
    double sum = 0;
    for (int i = 0; i < vec.size(); i++) {
        sum += vec(i)*vec(i);
    }
    return sqrt(sum);
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "gripper_estimation");
    ros::NodeHandle nh;
    int publish_rate = 500;
    ros::Rate loop_rate(publish_rate);
    ros::Rate sleep_rate(1);

    ros::AsyncSpinner spinner(1);
    spinner.start();


    string robot_name; 
    // ros::param::get("/robot_in_use", robot_name);
    Ur_TrajectoryExecution ur5e_traj(nh, UR_TYPE::UR5e, "left", loop_rate);
    // ur5e.setWorldToBL((VectorXd(6) << 0, 0, 0, 0, 0, 3.142).finished());

    PayLoad_LeastSquare payload(nh, loop_rate, "/ft_sensor");
    

    


    VectorXd pos1(6);
    pos1 << 0.414, 0.453, 0.413, 3.14, 0.0, 3.14;
    VectorXd pos3(6);
    VectorXd pos4(6);

   
    ur5e_traj.movePoly5(pos1, 5, 1/double(publish_rate));
    ros::Duration(1).sleep();
    // payload.computeBiasUsingGripperMass(0.0);
    ros::Duration(1).sleep();
    payload.registerFTValues(pos1.tail(3));


    VectorXd pos2(6);
    pos2 << 0.414, 0.453, 0.413, 3.14, 1.57, 3.14;
    ur5e_traj.movePoly5(pos2, 5, 1/double(publish_rate));
    ros::Duration(1).sleep();
    payload.registerFTValues(pos2.tail(3));
    
    VectorXd J_pos3 = ur5e_traj.getJointPositions();
    
    J_pos3(5) = 0.707;
    ur5e_traj.MoveJoint(J_pos3);
    ros::Duration(3).sleep();
    payload.registerFTValues(ur5e_traj.getForwardKinematics().tail(3));

    J_pos3(5) = 0.0;
    ur5e_traj.MoveJoint(J_pos3);
    ros::Duration(3).sleep();
    payload.registerFTValues(ur5e_traj.getForwardKinematics().tail(3));

    payload.CoG_Identification();





    while (ros::ok()) {
        VectorXd V_fwrd = ur5e_traj.getForwardKinematics();


        cout << payload.getFTVector(V_fwrd(3), V_fwrd(4), V_fwrd(5), false).transpose() << endl;

        cout << payload.getFTVector(V_fwrd(3), V_fwrd(4), V_fwrd(5), true).transpose() << endl;
        
        
        // cout << "Force in Base frame: " << endl << ForceTorque.head(3) << endl;
        
        sleep_rate.sleep();
    }


    return 0;
}


    // SensorReader urFTSensor(nh, publish_rate, "/right/wrench");
    // urFTSensor.addMass_and_CenterOfMass(0.25, 0, 0, 0);
    // urFTSensor.addMass_and_CenterOfMass(0.099, 0, 0, 0);
    // urFTSensor.addMass_and_CenterOfMass(1.121, 0, 0, 0);
    // urFTSensor.setUseCompensateGravity(true);
    // urFTSensor.setRotMatrix_sb(V_fwrd(3), V_fwrd(4), V_fwrd(5));
    // urFTSensor.setUseMovingAverageFilter(true);

    // urFTSensor.setSensorBias();

    // urFTSensor.setSensorBias_Using2RobotPos();
    // urFTSensor.setUseCompensateGravity(true);