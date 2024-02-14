#pragma once

#include <iostream>
#include <cmath>
#include <string>
#include <eigen3/Eigen/Dense>
#include <ur_pers_lib_pkg/3DTransformations/Homogeneus.hpp>
#include <ur_pers_lib_pkg/UrToolKit/Ur_Utils/Ur_DenavitHartenberg.hpp>
#include <ur_pers_lib_pkg/UrToolKit/Ur_Utils/UR_Types.hpp>
#include "FindNearestConfiguration.hpp"
#include <vector>


double Degree_To_Radiant(double deg);
double Radiant_To_Degree(double rad);
double* eigenToDoubleArray(const Eigen::Matrix4d& matrix);
Eigen::MatrixXd arrayToMatrixXd(double* array, int num_rows, int num_cols);






namespace Universal_Robot {

  
  Eigen::VectorXd correctJointGoal(Eigen::VectorXd joint_act_pos, Eigen::VectorXd joint_goal);
  class Kinematics : private DH_Parameters {

    public:

      Kinematics(UR_TYPE robot_name);

      void setWorldToBL(Eigen::VectorXd w_t_bl); //world ---> base_link
      void setEeToTcp(Eigen::VectorXd ee_t_tcp); //toool0_controller ---> TCP

      Eigen::Matrix4d forward_asHomogeneous(Eigen::VectorXd q);
      Eigen::VectorXd forward_asVector(Eigen::VectorXd q);
      Eigen::VectorXd foward_NJoint(Eigen::VectorXd q, int joint_n);

      Eigen::MatrixXd inverse_kin_WToTcp(Eigen::VectorXd ee_pos);
      Eigen::MatrixXd inverse_kin_BlToEE(Eigen::VectorXd ee_pos);


      bool getMinNormSolution(Eigen::VectorXd& joint_values, Eigen::VectorXd target_ee_pos, bool correct_sign=true);
      std::vector<Eigen::VectorXd> cartTraj_To_JointTraj(std::vector<double>& pos_x, std::vector<double>& pos_y, std::vector<double>& pos_z, std::vector<double>& rot_x, std::vector<double>& rot_y, std::vector<double>& rot_z, Eigen::VectorXd q0);
      
      int getNumOfSols();

    private:


      int __inverse(const double* T, double* q_sols, double q6_des=0.0);
      Eigen::Matrix4d __forward(Eigen::VectorXd q);

      Eigen::Matrix4d _DH_MatrixTF(double a, double alpha, double d, double theta);

      Eigen::Matrix4d T_6th_to_ee; // matrix that adds the rotation of the end-effector
      Eigen::Matrix4d T_bl_to_base, T_world_to_bl, T_ee_to_tcp; // attenzione in simulaziione prende il base_link
      


      int num_of_sols;  
      
  };

}
