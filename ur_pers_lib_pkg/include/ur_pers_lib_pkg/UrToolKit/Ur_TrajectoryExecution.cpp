#include "Ur_TrajectoryExecution.hpp"

using namespace std;
using namespace Eigen;

namespace Universal_Robot{

    Ur_TrajectoryExecution::Ur_TrajectoryExecution(ros::NodeHandle& nh, Universal_Robot::UR_TYPE ur_type, std::string prefix, ros::Rate& rate)
    : Ur_JointGroupController(nh, ur_type, prefix, rate)
    , trajectory3D()
    , rate(rate)
    {
        this->traj_pub_visu_rqt = nh.advertise<geometry_msgs::Pose>("ur_trajectory", 100);
    }
    

    Ur_TrajectoryExecution::~Ur_TrajectoryExecution(){}

    void Ur_TrajectoryExecution::setWorld_To_Baselink(double x, double y, double z, double roll, double pitch, double yaw){
        VectorXd arm_world_to_baselink(6);
        arm_world_to_baselink << x, y, z, roll, pitch, yaw;
        Ur_JointGroupController::setWorldToBL(arm_world_to_baselink);
    }

    void Ur_TrajectoryExecution::setEndEff_To_ToolCenterPoint(double x, double y, double z, double roll, double pitch, double yaw){
        VectorXd arm_ee_to_tcp(6);
        arm_ee_to_tcp << x, y, z, roll, pitch, yaw;
        Ur_JointGroupController::setEeToTcp(arm_ee_to_tcp);
    }

    void Ur_TrajectoryExecution::pubTrajectory(){
        geometry_msgs::Pose pose_pub;
        for (int i = 0; i < this->trajectory.size(); i++){
            Ur_JointGroupController::MovePtP(this->trajectory[i]);
            pose_pub.position.x = this->trajectory[i](0);
            pose_pub.position.y = this->trajectory[i](1);
            pose_pub.position.z = this->trajectory[i](2);
            traj_pub_visu_rqt.publish(pose_pub);
            rate.sleep();
        }

    }


    void Ur_TrajectoryExecution::moveTrapezoidal(VectorXd end_pose, double acc_time, double dec_time, double traj_period, double discretizzation)
    {
        VectorXd start_pose = this->getForwardKinematics();

        trajectory3D.setTrajectoryType("trapezoidal", traj_period, discretizzation, acc_time, dec_time);
        
        this->trajectory.clear();
        this->trajectory = trajectory3D.getPosRotTrajPoints(start_pose, end_pose);
        
        this->pubTrajectory(); // todo

    }

    void Ur_TrajectoryExecution::movePoly5(VectorXd end_pose, double T, double discretizzation)
    {
        VectorXd start_pose = this->getForwardKinematics();

        trajectory3D.setTrajectoryType("poly5", T, discretizzation);
        this->trajectory.clear();
        this->trajectory = trajectory3D.getPosRotTrajPoints(start_pose, end_pose);
        
        this->pubTrajectory(); 
    }



} // namespace