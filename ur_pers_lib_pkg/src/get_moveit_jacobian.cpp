#include <ros/ros.h>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_model_and_robot_state_tutorial");
  ros::AsyncSpinner spinner(1);
  spinner.start();


  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("manipulator");
  //dim6 all to zero
  std::vector<double> joint_values = {-2.548871977575965, -2.0248609214314786, -1.574481982947777, -0.29908405377582326, 5.536207367575946, 0.6055001576352761};






        kinematic_state->setJointGroupPositions(joint_model_group, joint_values);
        // kinematic_state->enforceBounds();
        const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
          for (std::size_t i = 0; i < joint_names.size(); ++i)
          {
            ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
          }
          Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
        Eigen::MatrixXd jacobian;
        kinematic_state->getJacobian(joint_model_group, kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                                  reference_point_position,
                                      jacobian);
        ROS_INFO_STREAM("Jacobian: \n" << jacobian << "\n");

  ros::shutdown();
  return 0;
}
