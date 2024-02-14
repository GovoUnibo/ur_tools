#pragma once

#include <eigen3/Eigen/Dense>
#include <vector>
#include <iostream>
#include <string>
#include <ur_pers_lib_pkg/UrToolKit/Ur_kins/Kinematics.hpp>
#include <ur_pers_lib_pkg/UrToolKit/Ur_Utils/Ur_JointNames.hpp>
#include <ur_pers_lib_pkg/Ros_Controllers/JointGroupPubSub.hpp>


namespace Universal_Robot{
    
    class Ur_JointGroupController : private  Kinematics  {

        public:
            Ur_JointGroupController(ros::NodeHandle& nh, Universal_Robot::UR_TYPE ur_type, std::string prefix, ros::Rate& rate);
            ~Ur_JointGroupController();

            
            Eigen::VectorXd getForwardKinematics();
            Eigen::VectorXd getJointPositions();
            Eigen::VectorXd getNearestJointSolution(Eigen::VectorXd pose);
            void MoveJoint(Eigen::VectorXd joint_goal);
            void pubAndWaitInposition(Eigen::VectorXd joint_goal, double seconds_to_timeout= 10);
            void MovePtP(Eigen::VectorXd pose);

            using Kinematics::setWorldToBL;
            using Kinematics::setEeToTcp;
            


        private:
            JointNames jointNamesList;
            JointGroupPubSub robotController;

    };
}
