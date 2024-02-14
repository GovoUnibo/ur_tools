#include <ur_pers_lib_pkg/UrToolKit/Ur_ComplianceControl.hpp>

using namespace std;
using namespace Eigen;
using namespace Controller;

namespace Universal_Robot{
    ComplianceController::ComplianceController(ros::NodeHandle& nh, Universal_Robot::UR_TYPE ur_type,  std::string prefix, std::string ft_sensor_topic, ros::Rate& rate)
        : Ur_JointGroupController(nh, ur_type, prefix, rate)
        , FT_Sensor(nh, rate, ft_sensor_topic)
        , Admittance(rate)
        {       
            std::cout << "ComplianceController initialized." << std::endl;
            VectorXd base_link_rotation = (VectorXd(6) << 0, 0, 0, 0, 0, -1.57).finished();
            Ur_JointGroupController::setWorldToBL(base_link_rotation);
        }

    ComplianceController::~ComplianceController(){
        std::cout << "ComplianceController destroyed." << std::endl;
        ros::waitForShutdown();
    }

    void ComplianceController::holdFixedPosition(VectorXd target_pose){
        actual_force = FT_Sensor.getWrenchInBaseFrame();
        Admittance::fixRobotSetPoint(target_pose);
        VectorXd set_point = Admittance::computeAdmittance(actual_force);
        
        Ur_JointGroupController::MovePtP(set_point);
        
    }




} // namespace