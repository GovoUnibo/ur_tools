#pragma once

#include <string>
#include <vector>
#include <iostream>
#include "UR_Types.hpp"

namespace Universal_Robot
{
    

    class JointNames{
        public:
            JointNames();
            JointNames(UR_TYPE);
            JointNames(std::string custom_prefix);

            ~JointNames();

            std::vector<std::string> getList();
            bool hasJoint(std::string joint_name);
        private:
            std::string joint_1_name = "shoulder_pan_joint";
            std::string joint_2_name = "shoulder_lift_joint";
            std::string joint_3_name = "elbow_joint";
            std::string joint_4_name = "wrist_1_joint";
            std::string joint_5_name = "wrist_2_joint";
            std::string joint_6_name = "wrist_3_joint";
            std::string prefix = "";
            std::vector<std::string> joint_names;
    };
    
} // namespace Universal_Robot
