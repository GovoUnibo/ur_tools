#include <ur_pers_lib_pkg/UrToolKit/Ur_TrajectoryExecution.hpp>

using namespace std;
using namespace Eigen;
using namespace Universal_Robot;

int main(int argc, char** argv){
    ros::init(argc, argv, "test");
    ros::NodeHandle nh;
    int publish_rate = 500;
    ros::Rate rate(publish_rate);

    string robot_name;    
    ros::param::get("/robot_in_use", robot_name);

    Ur_TrajectoryExecution dual_arm(nh, UR_TYPE::UR5e, robot_name, rate);
    ros::Duration(1).sleep();

    dual_arm.setWorld_To_Baselink(0.0, 0.0, 0.0, 0, 0, -1.57); // se voglio fare uscire l'asse x dal connettore
    dual_arm.setEndEff_To_ToolCenterPoint(0.0, 0.0, 0.23, 0, 0, 0); // rosrun tf tf_echo /right_tool0 /right_Tcp
    VectorXd actual_pose = dual_arm.getForwardKinematics();

    actual_pose(0) = 0.25;
    actual_pose(1) = 0.0;
    actual_pose(2) = 0.3;
    actual_pose(3) = 3.14;
    actual_pose(4) = 0.4;
    actual_pose(5) = 1.57;

    


    // dual_arm.movePoly5(dual_arm.getForwardKinematics(), actual_pose, 10, 1/double(publish_rate));


    // actual_pose(0) = 0.3;
    // actual_pose(1) = 0.2;
    // actual_pose(2) = 0.5;
    // actual_pose(3) = 3.14;
    // actual_pose(4) = 0;
    // actual_pose(5) = 1.57;

    // dual_arm.moveTrapezoidal(dual_arm.getForwardKinematics(), actual_pose, 0.1, 2, 20, 1/double(publish_rate));

    // cout << dual_arm.getEndEffectorPosition().transpose() << endl;
    return 0;
}

