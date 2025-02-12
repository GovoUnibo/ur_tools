#include "Kinematics.hpp"

#include <math.h>
#include <stdio.h>


using namespace Eigen;
using namespace std;
using namespace HomogeneousMatrix;

double Degree_To_Radiant(double deg){
  return (deg * M_PI / 180.0);
}

double Radiant_To_Degree(double rad){
  return (rad * 180.0 / M_PI);
}

double* eigenToDoubleArray(const Eigen::Matrix4d& matrix) {
    double* _T = new double[16];

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            _T[i * 4 + j] = matrix(i, j);
        }
    }

    return _T;
}

Eigen::MatrixXd arrayToMatrixXd(double* array, int num_rows, int num_cols) {
    Eigen::MatrixXd matrix(num_rows, num_cols);
    for (int i = 0; i < num_rows; i++) {
        for (int j = 0; j < num_cols; j++) {
            matrix(i, j) = array[i * num_cols + j];
        }
    }
    return matrix;
}


namespace {
  const double ZERO_THRESH = 0.00000001;
  int SIGN(double x){
    return (x > 0) - (x < 0);
  }
  const double PI = M_PI;
}

namespace Universal_Robot{
  /*
                                                                                              
  
  ########   #     #     ######     
  #      #   #     #     #     #    
  #      #   #     #     #     #       
  ########   #     #     ######       
  #          #     #     #     #
  #          #     #     #     #                                                                                                                                                                                                                                                                                                     
  #           #####      ######       
                                                                        
                                                                                                
  */
// la cinematica inversa torna sempre valori che sono positivi, ma la posizione attuale potrebbe essere negativa
// quindi devo convertire i valori positivi in negativi nel caso in cui la posizione attuale la differenza tra i due valori sia maggiore di PI
  Eigen::VectorXd correctJointGoal(Eigen::VectorXd joint_act_pos, Eigen::VectorXd joint_goal){

    for (int i = 0; i < joint_goal.size(); ++i){
      if (joint_goal[i] - joint_act_pos[i] > M_PI){
        joint_goal[i] -= 2*M_PI;
      }
      else if (joint_goal[i] - joint_act_pos[i] < -M_PI){
        joint_goal[i] += 2*M_PI;
      }
    }
    return joint_goal;
  }


  Kinematics::Kinematics(UR_TYPE robot_name)
    : DH_Parameters(robot_name)
    {

      // this->T_6th_to_ee = this->T_6th_to_ee * Vector_To_HomogeneousMatrix(0,0,0, 1.571, 0.000, 1.571); //rosrun  tf tf_echo /ur5e_flange /ur5e_wrist_3_link
      // this->T_6th_to_ee <<  0, -1,  0,  0,
      //                       0,  0, -1,  0,
      //                       1,  0,  0,  0,
      //                       0,  0,  0,  1;

      this->T_bl_to_base <<  -1,  0,  0,  0,
                              0, -1,  0,  0,
                              0,  0,  1,  0,
                              0,  0,  0,  1;
      this->T_6th_to_ee = Vector_To_HomogeneousMatrix(0,0,0, -M_PI/2, 0, -M_PI/2);// trasforma il link 6 della cinematica con i dh param in quello wrist_3_link
      // this->T_tool0cont_to_tool0 = Vector_To_HomogeneousMatrix(0.010, 0.003, 0.032, 0.000, 0.007, -0.004); //offset tra tool0 e tool0_controller

      this->T_world_to_bl = Matrix4d::Identity();
      this->T_ee_to_tcp = Matrix4d::Identity();



    }

  void Kinematics::setWorldToBL(VectorXd w_t_bl){
    // rosrun tf tf_echo /world /base_link
    this->T_world_to_bl = Vector_To_HomogeneousMatrix(w_t_bl(0), w_t_bl(1), w_t_bl(2), w_t_bl(3), w_t_bl(4), w_t_bl(5));
  }

  void Kinematics::setEeToTcp(VectorXd ee_t_tcp){
    // rosrun tf tf_echo /ur5e_tool0 /ur5e_tcp_custom
    this->T_ee_to_tcp = Vector_To_HomogeneousMatrix(ee_t_tcp(0), ee_t_tcp(1), ee_t_tcp(2), ee_t_tcp(3), ee_t_tcp(4), ee_t_tcp(5));
  }


  VectorXd Kinematics::foward_NJoint(VectorXd q, int joint_n){
    //calcolare la cinematica diretta usando la matrice di trasformazione DH
    Matrix4d T = Matrix4d::Identity();
    for (int i = 0; i < joint_n; ++i)
      T = T * DH_Parameters::_DH_MatrixTF(DH_Parameters::a_coef[i], DH_Parameters::alpha_coef[i], DH_Parameters::d_coef[i], q[i]);

    
    T = this->T_world_to_bl * T;

    return HomogeneousMatrix_To_Vector(T);

  }

  Matrix4d Kinematics::forward_asHomogeneous(VectorXd q){
    return this->__forward(q);
  }

  VectorXd Kinematics::forward_asVector(VectorXd q){
    return HomogeneousMatrix_To_Vector(this->__forward(q));
  }


  MatrixXd Kinematics::inverse_kin_WToTcp(Eigen::VectorXd ee_pos){

    Matrix4d T = Vector_To_HomogeneousMatrix(ee_pos(0), ee_pos(1), ee_pos(2), ee_pos(3), ee_pos(4), ee_pos(5));

    T = this->T_world_to_bl.inverse() * T *this->T_ee_to_tcp.inverse() * this->T_6th_to_ee.inverse();

    double* _T = eigenToDoubleArray(T);

    double q_ik[8*6];

    this->num_of_sols = Kinematics::__inverse(_T, q_ik);

    return arrayToMatrixXd(q_ik, 8, 6);

}

  MatrixXd Kinematics::inverse_kin_BlToEE(Eigen::VectorXd ee_pos){

    // return this->inverse_kin(Vector_To_HomogeneousMatrix(ee_pos(0), ee_pos(1), ee_pos(2), ee_pos(3), ee_pos(4), ee_pos(5)));

    Matrix4d T = Vector_To_HomogeneousMatrix(ee_pos(0), ee_pos(1), ee_pos(2), ee_pos(3), ee_pos(4), ee_pos(5));

    T =  T * this->T_6th_to_ee.inverse();

    double* _T = eigenToDoubleArray(T);

    double q_ik[8*6];

    this->num_of_sols = Kinematics::__inverse(_T, q_ik);

    // std::cout << "num_of_sols: " << this->num_of_sols << std::endl;

    return arrayToMatrixXd(q_ik, 8, 6);

  }


  int Kinematics::getNumOfSols(){ return this->num_of_sols;} // end getNumOfSols

  bool  Kinematics::getMinNormSolution(VectorXd& joint_values, VectorXd target_ee_pos, bool correct_sign){
    MatrixXd solutions = this->inverse_kin_WToTcp(target_ee_pos);
    this->num_of_sols = this->getNumOfSols();
    
    VectorXd positive_current_joint_values = joint_values;
    //convert current_joint_values to positive values, since the inverse kinematics returns only positive values
    for (int i=0; i<6; i++){
      if (positive_current_joint_values(i)<0)
        positive_current_joint_values(i) = 2*M_PI + positive_current_joint_values(i);
    }

    if (num_of_sols > 0){
        MatrixXd joint_variation(num_of_sols,6);

      

      for (int i = 0; i < num_of_sols; i++){
        for (int j = 0; j < 6; ++j){
          joint_variation(i,j) = fabs(positive_current_joint_values(j) - solutions(i,j));
          if (joint_variation(i,j) > M_PI)
            joint_variation(i,j) = 2*M_PI - joint_variation(i,j);
        }

      }
      
      
      
      //select the solution closest to the current joint values
      int sol_selected = trovaIndiceConfigurazioneVicina(positive_current_joint_values, solutions, VectorXd::Ones(6));
    
      // cout << "sols: " << num_of_sols << "sol_selected" <<sol_selected <<endl;
      // cout << "sol_selected: " << solutions.row(sol_selected) << endl;
      // cout << "corrected sol " << correctJointGoal(joint_values, solutions.row(sol_selected)) << endl;
      if (correct_sign)
        joint_values =  correctJointGoal(joint_values, solutions.row(sol_selected));
      
      return  num_of_sols>0;
    } // if (num_of_sols > 0)
    else
      return false;
    
  } // end computeMinNormSolution



  /*
                                                                                              
  
  ########   ######   #####  #      #             
  #      #   #     #    #    #      #    
  #      #   #     #    #    #      #       
  ########   ######     #    #      #    
  #          #   #      #     #    #
  #          #    #     #      #  #
  #          #     #  #####     #


*/




  Matrix4d Kinematics::__forward(VectorXd q) {
    Matrix4d T;

    double s1 = sin(q(0)),  c1 = cos(q(0));
    double q23 = q(1), q234 = q(1), s2 = sin(q(1)), c2 = cos(q(1));
    double s3 = sin(q(2)), c3 = cos(q(2)); q23 += q(2); q234 += q(2); 
    double s4 = sin(q(3)), c4 = cos(q(3)); q234 += q(3);
    double s5 = sin(q(4)), c5 = cos(q(4));
    double s6 = sin(q(5)), c6 = cos(q(5)); 
    double s23 = sin(q23), c23 = cos(q23);
    double s234 = sin(q234), c234 = cos(q234);

    T(0,0) = c234*c1*s5 - c5*s1;
    T(0,1) = c6*(s1*s5 + c234*c1*c5) - s234*c1*s6; 
    T(0,2) = -s6*(s1*s5 + c234*c1*c5) - s234*c1*c6;
    T(0,3) = d6*c234*c1*s5 - a3*c23*c1 - a2*c1*c2 - d6*c5*s1 - d5*s234*c1 - d4*s1; 
    T(1,0) = c1*c5 + c234*s1*s5;
    T(1,1) = -c6*(c1*s5 - c234*c5*s1) - s234*s1*s6; 
    T(1,2) = s6*(c1*s5 - c234*c5*s1) - s234*c6*s1; 
    T(1,3) = d6*(c1*c5 + c234*s1*s5) + d4*c1 - a3*c23*s1 - a2*c2*s1 - d5*s234*s1; 
    T(2,0) = -s234*s5; 
    T(2,1) = -c234*s6 - s234*c5*c6; 
    T(2,2) = s234*c5*s6 - c234*c6; 
    T(2,3) = d1 + a3*s23 + a2*s2 - d5*(c23*c4 - s23*s4) - d6*s5*(c23*s4 + s23*c4); 
    T(3,0) = 0.0; 
    T(3,1) = 0.0; 
    T(3,2) = 0.0; 
    T(3,3) = 1.0;
  

    // return this->T_bl_to_base.inverse()*T*this->T_6th_to_ee;//real?
    // return T*this->T_6th_to_ee; //simulation

    return this->T_world_to_bl * T * this->T_6th_to_ee * this->T_ee_to_tcp;

  }



  int Kinematics::__inverse(const double* T, double* q_sols, double q6_des) {
    int num_sols = 0;
    double T02 = -*T; T++; double T00 =  *T; T++; double T01 =  *T; T++; double T03 = -*T; T++; 
    double T12 = -*T; T++; double T10 =  *T; T++; double T11 =  *T; T++; double T13 = -*T; T++; 
    double T22 =  *T; T++; double T20 = -*T; T++; double T21 = -*T; T++; double T23 =  *T;

    ////////////////////////////// shoulder rotate joint (q1) //////////////////////////////
    double q1[2];
    {
      double A = d6*T12 - T13;
      double B = d6*T02 - T03;
      double R = A*A + B*B;
      if(fabs(A) < ZERO_THRESH) {
        double div;
        if(fabs(fabs(d4) - fabs(B)) < ZERO_THRESH)
          div = -SIGN(d4)*SIGN(B);
        else
          div = -d4/B;
        double arcsin = asin(div);
        if(fabs(arcsin) < ZERO_THRESH)
          arcsin = 0.0;
        if(arcsin < 0.0)
          q1[0] = arcsin + 2.0*PI;
        else
          q1[0] = arcsin;
        q1[1] = PI - arcsin;
      }

      else if(fabs(B) < ZERO_THRESH) {
        double div;
        if(fabs(fabs(d4) - fabs(A)) < ZERO_THRESH)
          div = SIGN(d4)*SIGN(A);
        else
          div = d4/A;
        double arccos = acos(div);
        q1[0] = arccos;
        q1[1] = 2.0*PI - arccos;
      }

      else if(d4*d4 > R) {
        return num_sols;
      }

      else {
        double arccos = acos(d4 / sqrt(R)) ;
        double arctan = atan2(-B, A);
        double pos = arccos + arctan;
        double neg = -arccos + arctan;
        if(fabs(pos) < ZERO_THRESH)
          pos = 0.0;
        if(fabs(neg) < ZERO_THRESH)
          neg = 0.0;
        if(pos >= 0.0)
          q1[0] = pos;
        else
          q1[0] = 2.0*PI + pos;
        if(neg >= 0.0)
          q1[1] = neg; 
        else
          q1[1] = 2.0*PI + neg;
      }
    }

    // std::cout << "q1[0]: " << q1[0] << " q1[1]: " << q1[1] << std::endl;
    ////////////////////////////////////////////////////////////////////////////////

    ////////////////////////////// wrist 2 joint (q5) //////////////////////////////
    double q5[2][2];
    {
      for(int i=0;i<2;i++) {
        double numer = (T03*sin(q1[i]) - T13*cos(q1[i])-d4);
        double div;
        if(fabs(fabs(numer) - fabs(d6)) < ZERO_THRESH)
          div = SIGN(numer) * SIGN(d6);
        else
          div = numer / d6;
        double arccos = acos(div);
        q5[i][0] = arccos;
        q5[i][1] = 2.0*PI - arccos;
      }
    }

    // std::cout << "q5[0][0]: " << q5[0][0] << " q5[0][1]: " << q5[0][1] << std::endl;
    // std::cout << "q5[1][0]: " << q5[1][0] << " q5[1][1]: " << q5[1][1] << std::endl;
    ////////////////////////////////////////////////////////////////////////////////

    {
      for(int i=0;i<2;i++) {
        for(int j=0;j<2;j++) {
          double c1 = cos(q1[i]), s1 = sin(q1[i]);
          double c5 = cos(q5[i][j]), s5 = sin(q5[i][j]);
          double q6;
          ////////////////////////////// wrist 3 joint (q6) //////////////////////////////
          if(fabs(s5) < ZERO_THRESH)
            q6 = q6_des;
          else {
            q6 = atan2(SIGN(s5)*-(T01*s1 - T11*c1), 
                        SIGN(s5)*(T00*s1 - T10*c1));
            if(fabs(q6) < ZERO_THRESH)
              q6 = 0.0;
            if(q6 < 0.0)
              q6 += 2.0*PI;
          }
          ////////////////////////////////////////////////////////////////////////////////

          double q2[2], q3[2], q4[2];
          ///////////////////////////// RRR joints (q2,q3,q4) ////////////////////////////
          double c6 = cos(q6), s6 = sin(q6);
          double x04x = -s5*(T02*c1 + T12*s1) - c5*(s6*(T01*c1 + T11*s1) - c6*(T00*c1 + T10*s1));
          double x04y = c5*(T20*c6 - T21*s6) - T22*s5;
          double p13x = d5*(s6*(T00*c1 + T10*s1) + c6*(T01*c1 + T11*s1)) - d6*(T02*c1 + T12*s1) + 
                        T03*c1 + T13*s1;
          double p13y = T23 - d1 - d6*T22 + d5*(T21*c6 + T20*s6);

          double c3 = (p13x*p13x + p13y*p13y - a2*a2 - a3*a3) / (2.0*a2*a3);

          if(fabs(fabs(c3) - 1.0) < ZERO_THRESH)
            c3 = SIGN(c3);
          else if(fabs(c3) > 1.0) {
            continue;
          }
          double arccos = acos(c3);
          q3[0] = arccos;
          q3[1] = 2.0*PI - arccos;
          double denom = a2*a2 + a3*a3 + 2*a2*a3*c3;
          double s3 = sin(arccos);
          double A = (a2 + a3*c3), B = a3*s3;
          q2[0] = atan2((A*p13y - B*p13x) / denom, (A*p13x + B*p13y) / denom);
          q2[1] = atan2((A*p13y + B*p13x) / denom, (A*p13x - B*p13y) / denom);
          double c23_0 = cos(q2[0]+q3[0]);
          double s23_0 = sin(q2[0]+q3[0]);
          double c23_1 = cos(q2[1]+q3[1]);
          double s23_1 = sin(q2[1]+q3[1]);
          q4[0] = atan2(c23_0*x04y - s23_0*x04x, x04x*c23_0 + x04y*s23_0);
          q4[1] = atan2(c23_1*x04y - s23_1*x04x, x04x*c23_1 + x04y*s23_1);
          ////////////////////////////////////////////////////////////////////////////////
          for(int k=0;k<2;k++) {
            if(fabs(q2[k]) < ZERO_THRESH)
              q2[k] = 0.0;
            else if(q2[k] < 0.0) q2[k] += 2.0*PI;
            if(fabs(q4[k]) < ZERO_THRESH)
              q4[k] = 0.0;
            else if(q4[k] < 0.0) q4[k] += 2.0*PI;
            q_sols[num_sols*6+0] = q1[i];    q_sols[num_sols*6+1] = q2[k]; 
            q_sols[num_sols*6+2] = q3[k];    q_sols[num_sols*6+3] = q4[k]; 
            q_sols[num_sols*6+4] = q5[i][j]; q_sols[num_sols*6+5] = q6; 
            num_sols++;
          }
        }
      }
    }
    // std::cout << "num_sols: " << num_sols << std::endl;
    return num_sols;
  }







  // vector<VectorXd> Kinematics::cartTraj_To_JointTraj(vector<double>& pos_x, vector<double>& pos_y, vector<double>& pos_z, vector<double>& rot_x, vector<double>& rot_y, vector<double>& rot_z, VectorXd q0){
  //     vector<VectorXd> joint_traj;
      
  //     for (int i = 0; i < pos_x.size(); i++){
  //         VectorXd pos(6);
  //         pos << pos_x[i], pos_y[i], pos_z[i], rot_x[i], rot_y[i], rot_z[i];
  //         VectorXd joint_pos = ur_kinematics::correctJointGoal(q0, this->computeMinNormSolution(q0, pos));
  //         joint_traj.push_back(joint_pos);
  //     }

  //     joint_traj = this->__checkSafeTrajectory(joint_traj);
  //     return joint_traj;
  // }



}  // namespace ur_kinematics