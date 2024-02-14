#pragma once

#include "JS_Subscriber.hpp"
#include "JointGroupPub.hpp"
#include <ros/ros.h>
#include <vector>
#include <string>


class JointGroupPubSub : public JointGroupPub, public JointStateReader
{
    public:
        JointGroupPubSub(ros::NodeHandle& nh, const std::vector<std::string>& joint_names, std::string pub_name, std::string sub_name);
        ~JointGroupPubSub();

        using JointGroupPub::jointGoalPub; // Rendiamo il metodo jointGoalPub delle classi base accessibile come pubblico
        using JointStateReader::getJointPositions; // Rendiamo il metodo getJointPositions delle classi base accessibile come pubblico
        using JointStateReader::inPosition; // Rendiamo il metodo inPosition delle classi base accessibile come pubblico

        void checkAndSendJointGoal(Eigen::VectorXd joint_goal);
        void pubAndWaitInposition(Eigen::VectorXd joint_goal, double seconds_to_timeout= 10);

        bool inPosition(double tolerance);
        void waitInposition(double seconds_to_timeout);

    private:
        ros::NodeHandle nh_;
        ros::Timer timer;
        ros::AsyncSpinner spinner; // Questo spinner è necessario per far funzionare il callback su timeout del ros::Timer
        void timerCallback(const ros::TimerEvent& event);
        
        bool _checkBroadMovement(Eigen::VectorXd current_joint_values,Eigen::VectorXd joint_goal);
        bool timeout_for_movement;
};