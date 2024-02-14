#include "Ur_JointNames.hpp"

using namespace std;

namespace Universal_Robot
{
    JointNames::JointNames(){}
    JointNames::JointNames(UR_TYPE robot_type){
       
        switch (robot_type){
            
            case UR5:
                this->prefix = "ur5_";
                break;
            case UR5e:
                this->prefix = "ur5e_";
                break;
            case UR10:
                this->prefix = "ur10_";
                break;
            case UR10e:
                this->prefix = "ur10e_";
                break;
            case UR3:
                this->prefix = "ur3_";
                break;
            default:
                cout << "\033[1;33m" << "" << "\033[0m" << endl;
                break;

        }
       

    }
    JointNames::JointNames(string custom_prefix)
    :prefix(custom_prefix)
    {}

    JointNames::~JointNames(){}
    
    vector<string> JointNames::getList(){

        vector<string> joint_names = {  this->prefix + this->joint_1_name, 
                                        this->prefix + this->joint_2_name, 
                                        this->prefix + this->joint_3_name, 
                                        this->prefix + this->joint_4_name, 
                                        this->prefix + this->joint_5_name, 
                                        this->prefix + this->joint_6_name
                                        };

        
         //print in verde i nomi dei giunti selezionati
        cout << "\033[1;32m" << "USING JOINT NAMES: " << "\033[0m" << endl;
        for (int i = 0; i < joint_names.size(); i++){
            cout << "\033[1;32m" << "Joint Name " << i << ": " "\033[0m";
            cout << "\033[1;32m" << joint_names[i] << "\033[0m" << endl;
        }
        cout << "\033[1;32m" << "------------------" << "\033[0m" << endl;
        return joint_names;
    }

} // namespace Universal_Robot

