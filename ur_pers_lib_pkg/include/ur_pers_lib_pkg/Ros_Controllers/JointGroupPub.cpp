#include "JointGroupPub.hpp"

using namespace Eigen;
using namespace std;
JointGroupPub::JointGroupPub(ros::NodeHandle& nh, int num_of_joints, std::string topic_name)
:pub_name(topic_name)
{
    this->command_pub = nh.advertise<std_msgs::Float64MultiArray>(topic_name, 100);
    this->array_msg.data.resize(num_of_joints);
    last_target_joint_pos = Eigen::VectorXd::Zero(6);
    cout << "\033[1;32m" << "JOINT GROUP PUBLISHER --> initialized. Publisher name: " << this->pub_name << "\033[0m" << endl;

}

JointGroupPub::~JointGroupPub()
{
    cout << "\033[1;32m" << "JOINT GROUP PUBLISHER --> destroyed. Publisher name: " << this->pub_name << "\033[0m" << endl;

    this->command_pub.shutdown();
}


void JointGroupPub::jointGoalPub(const Eigen::VectorXd& joint_goals){
    //command new joint values to the robot, using a joint group pos message
    if (joint_goals.size() != array_msg.data.size()) {
        ROS_ERROR("Mismatch in the number of joint goals and expected joints.");
        return;
    }
    this->last_target_joint_pos = joint_goals;

    for (size_t i = 0; i < joint_goals.size(); i++)
        array_msg.data[i] = joint_goals(i);

    // std::cout << "\n" << std::endl;

    this->command_pub.publish(array_msg);
}

VectorXd JointGroupPub::getLastGoalPublished() const {
    return this->last_target_joint_pos;
}

