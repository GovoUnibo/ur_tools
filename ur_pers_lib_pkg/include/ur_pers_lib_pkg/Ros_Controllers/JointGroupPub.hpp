#ifndef JOINT_GROUP_PUB_H
#define JOINT_GROUP_PUB_H

#include <ros/ros.h>
#include <iostream>
#include <string>
#include <eigen3/Eigen/Dense>
#include <std_msgs/Float64MultiArray.h>

class JointGroupPub
{
public:

    JointGroupPub(ros::NodeHandle& nh, int num_of_joints, std::string topic_name = "/joint_group_position_controller/command");
    ~JointGroupPub();
    // Go to a joint goal
    void jointGoalPub(const Eigen::VectorXd& joint_goals);
    Eigen::VectorXd getLastGoalPublished() const ;


private:
    Eigen::VectorXd last_target_joint_pos;
    ros::Publisher command_pub;
    std_msgs::Float64MultiArray array_msg;
    std::string pub_name;
};


#endif // JOINT_GROUP_PUB_H