#include <ur_pers_lib_pkg//UrToolKit/Ur_kins/Kinematics.cpp>



using namespace Universal_Robot;
using namespace Eigen;

int main(int argc, char** argv){

    Kinematics kinematics(UR_TYPE::UR5e);
    Eigen::VectorXd q(6);
    // q << Degree_To_Radiant(-90), Degree_To_Radiant(-44), Degree_To_Radiant(-101), Degree_To_Radiant(224), Degree_To_Radiant(90), Degree_To_Radiant(0);
    q << 1.25841, 4.56155, 1.3808, 1.51202, 2.57009, -2.47159;
    for (int i = 0; i < 6; i++){
        if (q(i) < 0)
            q(i) = q(i) + 2*M_PI;
        
    }
    // std::cout << "q: \n" << q << std::endl;
    

    // VectorXd vector = kinematics.forward_asVector(q);
    // std::cout << "vector: \n" << vector << std::endl; //ok
    
    // VectorXd vector2 = kinematics.foward_NJoint(q, 6);
    // std::cout << "vector2: \n" << vector2 << std::endl; //ok

    VectorXd target1(6);
    target1 << -0.2576, -0.6250, -1.1306,  0.0000,  2.5133,  0.0000;

    VectorXd target2(6);
    target2 << 0.25, -0.6250, -0.1306,  0.0000,  2.5133,  0.0000;
    
    MatrixXd q_sols1 = kinematics.inverse_kin_BlToEE(target1);
    // std::cout << "num of sols = " << kinematics.getNumOfSols() << std::endl; //ok

    std::cout << "q_sols: \n" << q_sols1 << std::endl; //ok

    MatrixXd q_sols2 = kinematics.inverse_kin_BlToEE(target2);
    // std::cout << "num of sols = " << kinematics.getNumOfSols() << std::endl; //ok
    std::cout << "q_sols: \n" << q_sols2 << std::endl; //ok
    
    
    return 0;
}