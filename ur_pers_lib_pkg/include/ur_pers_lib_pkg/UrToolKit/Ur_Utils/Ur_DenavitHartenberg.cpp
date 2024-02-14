#include "Ur_DenavitHartenberg.hpp"

using namespace Eigen;
using namespace std;

namespace Universal_Robot{

    DH_Parameters::DH_Parameters(UR_TYPE robot_type)
    : a_coef(6)
    , d_coef(6)
    , alpha_coef(6)
    {
        cout << "\033[1m" << "UNIVERSAL ROBOT Kinematics:" << "\033[0m" << endl;

        switch (robot_type)
            {
            case UR_TYPE::UR10:
                this->d1 =  0.1273;
                this->a2 = -0.612;
                this->a3 = -0.5723;
                this->d4 =  0.163941;
                this->d5 =  0.1157;
                this->d6 =  0.0922;
                cout << "\033[1;32m" << "\t UR10 parameters loaded" << "\033[0m" << endl;
                break;
            case UR_TYPE::UR10e:
                this->d1 =  0.1807;   
                this->a2 = -0.6127;
                this->a3 = -0.57155;
                this->d4 =  0.17415;
                this->d5 =  0.11985; 
                this->d6 =  0.11655; 
                cout <<  "\033[1;32m" << "\t UR10e parameters loaded" << "\033[0m" << endl;
                break;
            case UR_TYPE::UR5:
                this->d1 =  0.089159;
                this->a2 = -0.42500;
                this->a3 = -0.39225;
                this->d4 =  0.10915;
                this->d5 =  0.09465;
                this->d6 =  0.0823;
                cout << "\033[1;32m" << "\t UR5 parameters loaded" << "\033[0m" << endl;
                break;
            case UR_TYPE::UR3:
                this->d1 =  0.1519;
                this->a2 = -0.24365;
                this->a3 = -0.21325;
                this->d4 =  0.11235;
                this->d5 =  0.08535;
                this->d6 =  0.0819;
                cout << "\033[1;32m" << "\t UR3 parameters loaded" << "\033[0m" << endl;
                break;
            case UR_TYPE::UR5e:
                this->d1 =  0.1625;
                this->a2 = -0.42500;
                this->a3 = -0.39225;
                this->d4 =  0.1333;
                this->d5 =  0.0997;
                this->d6 =  0.0996;
                cout << "\033[1;32m" << "\t UR5e parameters loaded" << "\033[0m" << endl;
                break;

            default:
                
                cout << "\033[1;31m" << "Robot parameters not loaded" << "\033[0m" << endl;
                break;
        }// end switch

        this->a_coef << 0, this->a2, this->a3, 0, 0, 0;
        this->d_coef << this->d1, 0, 0, this->d4, this->d5, this->d6;
        this->alpha_coef << M_PI/2, 0, 0, M_PI/2, -M_PI/2, 0;


    } // end constructor
    
    DH_Parameters::~DH_Parameters(){}

    Matrix4d DH_Parameters::_DH_MatrixTF(double a, double alpha, double d, double theta){
        Matrix4d T;
        T << cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta),
            sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta),
                    0,              sin(alpha),              cos(alpha),            d,
                    0,                        0,                        0,            1;
        return T;
    }
} // namespace Universal_Robot