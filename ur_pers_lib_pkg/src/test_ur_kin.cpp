#include <RobotTools/RobotKinematicsUtils/Kinematics.hpp>



using namespace ur_kinematics;
using namespace Eigen;

int main(int argc, char** argv){

    Kinematics kinematics(UR5e_PARAMS);
    Eigen::VectorXd q(6);
    // q << Degree_To_Radiant(-90), Degree_To_Radiant(-44), Degree_To_Radiant(-101), Degree_To_Radiant(224), Degree_To_Radiant(90), Degree_To_Radiant(0);
    q << 1.25841, 4.56155, 1.3808, 1.51202, 2.57009, -2.47159;
    for (int i = 0; i < 6; i++){
        if (q(i) < 0)
            q(i) = q(i) + 2*M_PI;
        
    }
    std::cout << "q: \n" << q << std::endl;
    

    VectorXd vector = kinematics.forward_asVector(q);
    std::cout << "vector: \n" << vector << std::endl; //ok
    
    VectorXd vector2 = kinematics.foward_NJoint(q, 6);
    std::cout << "vector2: \n" << vector2 << std::endl; //ok
    
    
    return 0;
}