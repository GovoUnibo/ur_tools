#include <ur_pers_lib_pkg/UrToolKit/Ur_ComplianceControl.hpp>

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
    string ft_sensor_topic = "/S1_TangentalIndicator";
    ros::AsyncSpinner spinner(1);
    spinner.start();
   

    ComplianceController r_ur5e(nh, UR_TYPE::UR5e, robot_name, ft_sensor_topic, rate);
    VectorXd target_pose = r_ur5e.getForwardKinematics();
    while (ros::ok())
    {
    
        r_ur5e.holdFixedPosition(target_pose);
        rate.sleep();
    }
    
   

    return 0;

}