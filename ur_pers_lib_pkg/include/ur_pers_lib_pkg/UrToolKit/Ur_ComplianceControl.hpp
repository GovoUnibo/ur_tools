#pragma once

#include <eigen3/Eigen/Dense>
#include <vector>
#include <iostream>
#include <string>
#include <ur_pers_lib_pkg/Controllers/ComplianceControl.hpp>
#include <ur_pers_lib_pkg/UrToolKit/Ur_JointGroupController.hpp>
#include <ur_pers_lib_pkg/ForceSensor/FT_Reader/SensorReader.hpp>



namespace Universal_Robot{
    class ComplianceController : private Controller::Admittance, Ur_JointGroupController{
        public:
            
            ComplianceController(ros::NodeHandle& nh, Universal_Robot::UR_TYPE ur_type,  
                                 std::string prefix, std::string ft_sensor_topic, ros::Rate& rate);

            ~ComplianceController();

            void holdFixedPosition(Eigen::VectorXd);

            using Ur_JointGroupController::getForwardKinematics;


            
        private:
            
            SensorReader FT_Sensor;
            Eigen::VectorXd actual_force;


    };
}
