#pragma once



#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <string>
#include <map>
#include <eigen3/Eigen/Dense>
// #include <mutex>

class JointStateReader {
public:
    JointStateReader(ros::NodeHandle& nh, const std::vector<std::string>& joint_names, std::string topic_name);
    ~JointStateReader();
    Eigen::VectorXd getJointPositions();
    bool isInitialized() const;
    bool inPosition(Eigen::VectorXd goal, double tolerance);

private:
    bool initialized_;
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    Eigen::VectorXd current_joint_value;
    ros::NodeHandle& nh_;
    ros::Subscriber joint_state_sub_;
    std::vector<std::string> joint_names_;
    std::map<std::string, double> joint_states_map_;
    ros::AsyncSpinner spinner; // Questo spinner è necessario per far funzionare il callback su Joint statea
    // std::mutex joint_state_mutex_; // In questo modo, il mutex assicurerà che solo un thread alla volta possa accedere ai dati di JointStateReader, garantendo l'accesso sicuro durante il callback e altri metodi.
};