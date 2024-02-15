#include "JS_Subscriber.hpp"


using namespace Eigen;
using namespace std;

JointStateReader::JointStateReader(ros::NodeHandle& nh, const std::vector<std::string>& joint_names, std::string topic_name = "/joint_states")
    : nh_(nh)
    , joint_names_(joint_names)
    , current_joint_value(joint_names.size())
    , spinner(1)
{
    
    this->spinner.start();
    this->joint_states_map_.clear();
    for (const auto& joint_name : this->joint_names_)
        joint_states_map_[joint_name] = 0.0;

    this->initialized_ = false;
    std::cout << "\033[1;33m" << "Waiting for joint states with names: " << topic_name << "\033[0m" << std::endl;
    for (const auto& joint_name : this->joint_names_)
        std::cout << "\t - \033[1;33m" << joint_name << "\033[0m" << std::endl;
    sensor_msgs::JointState::ConstPtr initial_msg =  ros::topic::waitForMessage<sensor_msgs::JointState>(topic_name);
    this->initialized_ = true;
    if(initial_msg)
        jointStateCallback(initial_msg);

    //scrivi in verde che il messaggio è ricevuto

    std::cout << "\033[1;32m" << "Joint states received on topic: " << topic_name << "\033[0m" << std::endl;

    this->joint_state_sub_ = nh_.subscribe(topic_name, 1, &JointStateReader::jointStateCallback, this);

}

JointStateReader::~JointStateReader(){
    cout << "Press CTRL + C to terminate Joint State subscriber" << endl;
    ros::waitForShutdown();
    spinner.stop();
}


bool JointStateReader::isInitialized() const{
    return this->initialized_;
}


void JointStateReader::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    // std::lock_guard<std::mutex> lock(joint_state_mutex_);

    for (size_t i = 0; i < msg->name.size(); ++i)
    {

        const std::string& joint_name = msg->name[i];
        cout << "Joint name: " << joint_name << endl;
        auto it = this->joint_states_map_.find(joint_name);

        if (it != this->joint_states_map_.end()) {
            this->joint_states_map_[joint_name] = msg->position[i];
            std::cout << "Chiave: " << it->first << ", Valore: " << it->second << std::endl;
        }
        
    }


    for (size_t i=0; i < joint_names_.size(); i++)
        this->current_joint_value(i) = joint_states_map_[joint_names_[i]];
    
}// Il mutex viene rilasciato automaticamente qui quando lock esce dallo scope

VectorXd JointStateReader::getJointPositions() { // usa la map
    // std::lock_guard<std::mutex> lock(joint_state_mutex_);
    return this->current_joint_value;
}


bool JointStateReader::inPosition(Eigen::VectorXd goal, double tolerance){
        // std::lock_guard<std::mutex> lock(joint_state_mutex_);
        // each joint needs to respect the tolerance
        for (int i = 0; i < current_joint_value.size(); i++){
            if (std::abs(current_joint_value(i) - goal(i)) > tolerance)
                return false;

        }
        return true;
}