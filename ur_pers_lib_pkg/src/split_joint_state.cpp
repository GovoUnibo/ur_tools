#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <string>
#include <iostream>
#include "ur_pers_lib_pkg/UrToolKit/Ur_Utils/Ur_JointNames.hpp"
using namespace std;

namespace Universal_Robot {
    class JointStateSplitter {
    public:
        JointStateSplitter(const std::string& arg1, const std::string& arg2)
            : arg1_(arg1)
            , arg2_(arg2) 
            , robot1_joint_names(arg1_+ '_')
            , robot2_joint_names(arg2_+ '_')
            , rate(1000)
            {
            // Inizializzazione del nodo ROS
            ros::NodeHandle nh;
            string joint_state_name = "joint_states";
            // Sottoscrizione al topic joint_states
            jointStateSub = nh.subscribe(joint_state_name, 10, &JointStateSplitter::jointStateCallback, this);

            // Pubblicazione su due nuovi topic
            jointStatePub1 = nh.advertise<sensor_msgs::JointState>(arg1_ + '/' + joint_state_name, 10);
            jointStatePub2 = nh.advertise<sensor_msgs::JointState>(arg2_ + '/' + joint_state_name, 10);

            cout << "\033[1;32m" << "JointStateSplitter initialized: " << "\033[0m" << endl;
            cout << "\033[1;32m" << "Robot 1: " << joint_state_name << "\033[0m" << endl;
            cout << "\033[1;32m" << "Robot 2: " << joint_state_name << "\033[0m" << endl;
        }
        
        bool hasPrefix(const std::string& joint_name, const std::string& prefix) {
            return joint_name.find(prefix) == 0;
        }

        // Callback per il topic joint_states
        void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
            // Creazione di due nuovi messaggi JointState
            // add stamp


            sensor_msgs::JointState msg1, msg2;

            // Estrazione dei joint relativi al primo robot
            for (size_t i = 0; i < msg->name.size(); ++i) {
                if (robot1_joint_names.hasJoint(msg->name[i])) {
                    msg1.name.push_back(msg->name[i]);
                    msg1.position.push_back(msg->position[i]);
                    msg1.velocity.push_back(msg->velocity[i]);
                    msg1.effort.push_back(msg->effort[i]);
                    // jointStatePub1.publish(msg1);
                }
                
            }
            

            // Estrazione dei joint relativi al secondo robot
            for (size_t i = 0; i < msg->name.size(); ++i) {
                if (robot2_joint_names.hasJoint(msg->name[i])) {
                    msg2.name.push_back(msg->name[i]);
                    msg2.position.push_back(msg->position[i]);
                    msg2.velocity.push_back(msg->velocity[i]);
                    msg2.effort.push_back(msg->effort[i]);
                    // jointStatePub2.publish(msg2);
                }
            }

            if (this->hasPrefix(msg->name[0], arg1_) ){
                msg1.header = msg->header;
                msg1.header.stamp = ros::Time::now();
                jointStatePub1.publish(msg1);
            }
            
            else if (this->hasPrefix(msg->name[0], arg2_)){
                msg2.header = msg->header;
                msg2.header.stamp = ros::Time::now();
                jointStatePub2.publish(msg2);
            }
            
            rate.sleep();
            
            
        }

    private:
        ros::Subscriber jointStateSub;
        ros::Publisher jointStatePub1;
        ros::Publisher jointStatePub2;

        std::string arg1_;
        std::string arg2_;
        JointNames robot1_joint_names;
        JointNames robot2_joint_names;
        ros::Rate rate;
    };
} // namespace Universal_Robot

int main(int argc, char** argv) {
    // Inizializzazione del nodo ROS
    ros::init(argc, argv, "joint_state_splitter");
    ros::NodeHandle nh;

    // Controllo dei parametri
    if (argc != 3) {
        ROS_ERROR("Usage: joint_state_splitter <arg1> <arg2>");
        return 1;
    }

    // Creazione dell'oggetto JointStateSplitter utilizzando gli argomenti del lancio
    Universal_Robot::JointStateSplitter splitter(argv[1], argv[2]);

    // Avvio del ciclo di esecuzione ROS
    ros::spin();

    return 0;
}
