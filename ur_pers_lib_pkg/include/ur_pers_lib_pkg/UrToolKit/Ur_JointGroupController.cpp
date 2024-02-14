#include "Ur_JointGroupController.hpp"

using namespace std;
using namespace Eigen;

namespace Universal_Robot{

    Ur_JointGroupController::Ur_JointGroupController(ros::NodeHandle& nh, Universal_Robot::UR_TYPE ur_type, std::string prefix, ros::Rate& rate)
    : jointNamesList(prefix + "_") 
    , robotController(nh, jointNamesList.getList(), prefix + "/joint_group_pos_controller/command", prefix + "/joint_states")
    , Kinematics(ur_type)
    {
        std::cout << "Ur_JointGroupController initialized." << std::endl;
    }

    Ur_JointGroupController::~Ur_JointGroupController(){
        std::cout << "Ur_JointGroupController destroyed." << std::endl;
        ros::waitForShutdown();
    }


    VectorXd Ur_JointGroupController::getForwardKinematics(){
        return Kinematics::forward_asVector(robotController.getJointPositions());
    }

    VectorXd Ur_JointGroupController::getJointPositions(){
        return robotController.getJointPositions();
    }

    VectorXd Ur_JointGroupController::getNearestJointSolution(VectorXd pose){
        VectorXd q = robotController.getJointPositions();
        bool success = Kinematics::getMinNormSolution(q, pose);
        if (!success){
            // print in red no solution exists for a given pose
            std::cout << "\033[1;31m" << "ERROR: No solution exists for a given pose. " << "\033[0m" << std::endl;
            exit(0);
        }
        return q;
    }

    void Ur_JointGroupController::MoveJoint(VectorXd joint_goal){
        robotController.jointGoalPub(joint_goal);
    }
    void Ur_JointGroupController::pubAndWaitInposition(VectorXd joint_goal, double seconds_to_timeout){
        robotController.pubAndWaitInposition(joint_goal, seconds_to_timeout);
    }

    void Ur_JointGroupController::MovePtP(VectorXd pose){
        VectorXd joint_goal = this->getNearestJointSolution(pose);
        // cout << "actual joints: " << robotController.getJointPositions().transpose() << endl;
        // cout << "joint_goal: " << joint_goal.transpose() << endl;
        robotController.jointGoalPub(joint_goal);
    }

} // namespace