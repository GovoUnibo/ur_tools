#include "JointGroupPubSub.hpp"

using namespace Eigen;
using namespace std;

JointGroupPubSub::JointGroupPubSub(ros::NodeHandle& nh, const std::vector<std::string>& joint_names, std::string pub_name, std::string sub_name="/joint_states") 
: nh_(nh)
, JointGroupPub(nh, joint_names.size(), pub_name)
, JointStateReader(nh, joint_names, sub_name)
, timer(nh.createTimer(ros::Duration(5), &JointGroupPubSub::timerCallback, this))
, spinner(1)
{
    this->spinner.start();

    this->timer.stop();

    this->timeout_for_movement = false;

    // dire che joint group pose command è stato inizializzato con successo e dire il nome del publsher. scrivi in verde
}



JointGroupPubSub::~JointGroupPubSub(){}


void JointGroupPubSub::timerCallback(const ros::TimerEvent& event){
    //timer callback
    this->timeout_for_movement = true;
    std::cout << "\033[1;33m" << "TIMEOUT WARNING: Robot can't reach the position. " << "\033[0m" << std::endl;
    // exit(0);
}


void JointGroupPubSub::checkAndSendJointGoal(Eigen::VectorXd joint_goal){
    //publish the joint goal

    // this->_checkBroadMovement(JointStateReader::getJointPositions(), joint_goal);
    JointGroupPub::jointGoalPub(joint_goal);
}

void JointGroupPubSub::pubAndWaitInposition(Eigen::VectorXd joint_goal, double seconds_to_timeout){
    // Correggi l'obiettivo congiunto e pubblicalo
    // Pubblica l'obiettivo congiunto e attendi finché il robot è in posizione
    this->timeout_for_movement = false;
    timer.setPeriod(ros::Duration(seconds_to_timeout));
    timer.start();
    
    while (ros::ok() && !JointStateReader::inPosition(joint_goal, 0.001) && !(this->timeout_for_movement)){   //( {
        this->checkAndSendJointGoal(joint_goal);
        ros::spinOnce();
    }

    timer.stop();
}

bool JointGroupPubSub::inPosition(double tolerance){
    //check if the robot is in position with the last target joint position
    return JointStateReader::inPosition(JointGroupPub::getLastGoalPublished(), tolerance);
}

void JointGroupPubSub::waitInposition(double seconds_to_timeout){
    //wait until the robot is in position
    this->timeout_for_movement = false;
    timer.setPeriod(ros::Duration(seconds_to_timeout));
    timer.start();
    while (ros::ok() && !JointStateReader::inPosition(JointGroupPub::getLastGoalPublished(), 0.5) && !(this->timeout_for_movement)){
        ros::spinOnce();
    }
    timer.stop();
}




bool JointGroupPubSub::_checkBroadMovement(VectorXd current_joint_values, VectorXd joint_goal){
    // utilizzando una matrice di pesi controllare se il valore di giunto di un robot supera il valore di giunto di un altro robot di una certa soglia
    // se il valore di giunto supera la soglia allora store in una lista il valore di giunto e la soglia superata
    //pubblicare un messaggio di warning ed aspettare un cin, y PER ESEGUIRE, n PER NON ESEGUIRE 
    VectorXd weights = VectorXd::Ones(6);
    weights << 1, 1, 1, 0.5, 0.5, 0.1;
    vector <pair<int, double>> joint_threshold;
    weights = weights * M_PI; 
    double joint_variation;
    bool result = false;

    for (int i = 0; i < current_joint_values.size(); ++i){
        joint_variation = abs(current_joint_values[i] - joint_goal[i]);
        if (joint_variation > weights[i]){
            joint_threshold.push_back(make_pair(i, joint_variation));
        }   
    }

    if (joint_threshold.size() > 0){
            result = true;
        
            std::cout << "\033[1;33m" << "WARNING: WIDE movement detected" << "\033[0m" << endl;
            std::cout << "Joint threshold: " << endl;
            for (int i = 0; i < joint_threshold.size(); ++i)
                std::cout << "Joint " << joint_threshold[i].first + 1 << " is moving from " << current_joint_values[joint_threshold[i].first] 
                << " to " << joint_goal[joint_threshold[i].first] << " with a variation of "<< joint_threshold[i].second << endl;

       
                std::cout << "\033[1;4m" << "Do you want to continue? Press [y/n]" << "\033[0m" << endl;
                char c;
                std::cin >> c;
                if (c != 'y')
                exit(0);
        
    }

    return result;

}