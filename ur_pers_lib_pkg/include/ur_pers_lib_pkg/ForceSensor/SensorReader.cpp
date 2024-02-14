#include "SensorReader.hpp"
using namespace Eigen;

using namespace std;
using namespace RPY;

SensorReader::SensorReader(ros::NodeHandle& nh, ros::Rate sensor_update_rate_hz, std::string topic_name)
: publish_rate(sensor_update_rate_hz)
, spinner(1)
, lowPassFilter_fx(0.9) //alpha
, lowPassFilter_fy(0.9)
, lowPassFilter_fz(0.9)
, lowPassFilter_tx(0.9)
, lowPassFilter_ty(0.9)
, lowPassFilter_tz(0.9)
, moving_average_fx(10)
, moving_average_fy(10)
, moving_average_fz(10)
, moving_average_tx(10)
, moving_average_ty(10)
, moving_average_tz(10)
{
    spinner.start();
    ft_subscriber = nh.subscribe(topic_name, 100, &SensorReader::FT_sensor_Reading_Callback, this);
    //init publisher on "ft_sensor/filtered" topic
    ft_publisher = nh.advertise<geometry_msgs::WrenchStamped>("ft_sensor/data_write", 2);

    this->useLowPassFilter = false;
    position_matrix_wrt_sensor <<   0, 0, 0,
                                    0, 0, 0,
                                    0, 0, 0;
    weight_vector << 0, 0, 0;
    R_matrix << 1, 0, 0,
                0, 1, 0,
                0, 0, 1;

    ros::topic::waitForMessage<geometry_msgs::WrenchStamped>(topic_name);
    std::cout << "SensorReader initialized" << std::endl;
}

SensorReader::~SensorReader(){
    cout << "Press CTRL + C to terminate SensorReader" << endl;
    ros::waitForShutdown();
    spinner.stop();

}

//----------------------------------Filters-----------------------------------------------//

void SensorReader::setUseLowPassFilter(bool useLowPassFilter){
    this->useLowPassFilter = useLowPassFilter;
}

void SensorReader::setUseMovingAverageFilter(bool useMovingAverageFilter){
    this->useMovingAverageFilter = useMovingAverageFilter;
}

void SensorReader::setUseCompensateGravity(bool useCompensateGravity){
    this->useCompensateGravity = useCompensateGravity;
}


const ForceTorque_s SensorReader::_LowPassFilter(const ForceTorque_s ft){
        return ForceTorque_s(lowPassFilter_fx.filter(ft.F_x),
                            lowPassFilter_fy.filter(ft.F_y),
                            lowPassFilter_fz.filter(ft.F_z),
                            lowPassFilter_tx.filter(ft.T_x),
                            lowPassFilter_ty.filter(ft.T_y),
                            lowPassFilter_tz.filter(ft.T_z));

}

const ForceTorque_s SensorReader::_MovingAverageFilter(const ForceTorque_s& ft){
    return ForceTorque_s(moving_average_fx.filter(ft.F_x),
                        moving_average_fy.filter(ft.F_y),
                        moving_average_fz.filter(ft.F_z),
                        moving_average_tx.filter(ft.T_x),
                        moving_average_ty.filter(ft.T_y),
                        moving_average_tz.filter(ft.T_z));
}

//----------------------------------Callbacks---------------------------------------------//

void SensorReader::FT_sensor_Reading_Callback(const geometry_msgs::WrenchStamped& msg){
    

    // log the message and compensate gravity
    this->FT_measured = msg;
    
    if(useCompensateGravity)
        this->FT_measured += this->compensateGravity();

    if (useMovingAverageFilter)
        this->FT_measured = _MovingAverageFilter(FT_measured);

    if (useLowPassFilter)
        this->FT_measured = _LowPassFilter(FT_measured);
    // std::cout << "Fx Measured: " << FT_measured.F_x << std::endl;
    // std::cout << "Fy Measured: " << FT_measured.F_y << std::endl;
    // std::cout << "Fz Measured: " << FT_measured.F_z << std::endl;
    // std::cout << "Tx Measured: " << FT_measured.T_x << std::endl;
    // std::cout << "Ty Measured: " << FT_measured.T_y << std::endl;
    // std::cout << "Tz Measured: " << FT_measured.T_z << std::endl;
    // std::cout << "------------------------" << std::endl;

}

void SensorReader::publishFT(){
    geometry_msgs::WrenchStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.wrench.force.x =  this->FT_measured.F_x;
    msg.wrench.force.y =  this->FT_measured.F_y;
    msg.wrench.force.z =  this->FT_measured.F_z;
    msg.wrench.torque.x = this->FT_measured.T_x;
    msg.wrench.torque.y = this->FT_measured.T_y;
    msg.wrench.torque.z = this->FT_measured.T_z;
    ft_publisher.publish(msg);
}


//----------------------------------Getters-----------------------------------------------//

VectorXd SensorReader::getWrenchInSensorFrame(){
    return FT_measured.getAsEigVector();
}

VectorXd SensorReader::getWrenchInBaseFrame(double rot_ft_x, double rot_ft_y, double rot_ft_z){
    // R_matrix is the rotation matrix from sensor frame to base frame (assigned through setRotMatrix_sb)
    Matrix3d R = RPY_To_RotationMatrix(rot_ft_x, rot_ft_y, rot_ft_z).transpose();
    Vector3d force_tool = R * FT_measured.getForcesVector();
    Vector3d torque_tool = R * FT_measured.getTorquesVector();

    return (VectorXd(6) << force_tool(0), force_tool(1), force_tool(2), torque_tool(0), torque_tool(1), torque_tool(2)).finished();
}

//----------------------------------Gravity compensation----------------------------------//

void SensorReader::addMass_and_CenterOfMass(double mass, double pos_x=0, double pos_y=0, double pos_z=0){
    /* 
    pos_x, pos_y, pos_z: baricenter position of the link wrt sensor
    mass: link mass
    */
    //weight vector calculation
    this->link_masses.push_back(mass);
    double total_mass = 0;
    
    for(auto mass_ : link_masses)
        total_mass += mass_;

    this->weight_vector << 0, 0, total_mass * 9.81;
    
    //center of mass calculation
    this->vector_of_link_baricenter_position.push_back(Eigen::Vector3d(pos_x, pos_y, pos_z));

    double p_x = 0, p_y = 0, p_z = 0; // these are the position of the center of mass of the wrist
    for (auto i = 0; i < vector_of_link_baricenter_position.size(); i++){
        p_x += vector_of_link_baricenter_position[i](0) * link_masses[i];
        p_y += vector_of_link_baricenter_position[i](1) * link_masses[i];
        p_z += vector_of_link_baricenter_position[i](2) * link_masses[i];
    }
    p_x /= total_mass;
    p_y /= total_mass;
    p_z /= total_mass;


    // this->position_vector_wrt_sensor << p_x, p_y, p_z;
    this->position_matrix_wrt_sensor << 0, -p_z, p_y,
                                        p_z, 0, -p_x,
                                        -p_y, p_x, 0;

}


ForceTorque_s SensorReader::compensateGravity(){
    ForceTorque_s FT_gravity;

    Vector3d force_tool_gravity = R_matrix * weight_vector;
    FT_gravity.F_x = force_tool_gravity(0);
    FT_gravity.F_y = force_tool_gravity(1);
    FT_gravity.F_z = force_tool_gravity(2);
    // normal of force
    // std::cout << "Normal of force: " << force_tool_gravity.norm() << std::endl;

    Vector3d mom_tool_gravity = position_matrix_wrt_sensor* R_matrix * weight_vector;
    FT_gravity.T_x = mom_tool_gravity(0);
    FT_gravity.T_y = mom_tool_gravity(1);
    FT_gravity.T_z = mom_tool_gravity(2);
    // std::cout << "Fx gravity: " << FT_gravity.F_x << std::endl;
    // std::cout << "Fy gravity: " << FT_gravity.F_y << std::endl;
    // std::cout << "Fz gravity: " << FT_gravity.F_z << std::endl;
    // std::cout << "Tx gravity: " << FT_gravity.T_x << std::endl;
    // std::cout << "Ty gravity: " << FT_gravity.T_y << std::endl;
    // std::cout << "Tz gravity: " << FT_gravity.T_z << std::endl;
    // std::cout << "------------------------" << std::endl;

    return FT_gravity;
}

void SensorReader::setRotMatrix_sb(double rot_ft_x, double rot_ft_y, double rot_ft_z){  
    // matrice di rotazione che porta il sensore nelle coordiate della base
    R_matrix = RPY_To_RotationMatrix(rot_ft_x, rot_ft_y, rot_ft_z).transpose();
}





//--------------Operator overloading-----------------//

ForceTorque_s::ForceTorque_s(){
    F_x = 0;
    F_y = 0;
    F_z = 0;
    T_x = 0;
    T_y = 0;
    T_z = 0;
}

ForceTorque_s::ForceTorque_s(double F_x, double F_y, double F_z, double T_x, double T_y, double T_z){
    this->F_x = F_x;
    this->F_y = F_y;
    this->F_z = F_z;
    this->T_x = T_x;
    this->T_y = T_y;
    this->T_z = T_z;
}

ForceTorque_s::~ForceTorque_s(){}

void ForceTorque_s::operator=(const ForceTorque_s& ft){
    F_x = ft.F_x;
    F_y = ft.F_y;
    F_z = ft.F_z;
    T_x = ft.T_x;
    T_y = ft.T_y;
    T_z = ft.T_z;
}

void ForceTorque_s::operator=(const geometry_msgs::WrenchStamped& msg){
    F_x = msg.wrench.force.x;
    F_y = msg.wrench.force.y;
    F_z = msg.wrench.force.z;
    T_x = msg.wrench.torque.x;
    T_y = msg.wrench.torque.y;
    T_z = msg.wrench.torque.z;
}

void ForceTorque_s::operator=(const Eigen::VectorXd& vec){
    F_x = vec(0);
    F_y = vec(1);
    F_z = vec(2);
    T_x = vec(3);
    T_y = vec(4);
    T_z = vec(5);
}

ForceTorque_s operator+(const ForceTorque_s& ft, const ForceTorque_s& ft2){
    return ForceTorque_s(ft.F_x + ft2.F_x, ft.F_y + ft2.F_y, ft.F_z + ft2.F_z, ft.T_x + ft2.T_x, ft.T_y + ft2.T_y, ft.T_z + ft2.T_z);
}

ForceTorque_s operator-(const ForceTorque_s& ft, const ForceTorque_s& ft2){
    return ForceTorque_s(ft.F_x - ft2.F_x, ft.F_y - ft2.F_y, ft.F_z - ft2.F_z, ft.T_x - ft2.T_x, ft.T_y - ft2.T_y, ft.T_z - ft2.T_z);
}



void ForceTorque_s::operator+=(const ForceTorque_s& ft){
    F_x += ft.F_x;
    F_y += ft.F_y;
    F_z += ft.F_z;
    T_x += ft.T_x;
    T_y += ft.T_y;
    T_z += ft.T_z;
}

void ForceTorque_s::operator-=(const ForceTorque_s& ft){
    F_x -= ft.F_x;
    F_y -= ft.F_y;
    F_z -= ft.F_z;
    T_x -= ft.T_x;
    T_y -= ft.T_y;
    T_z -= ft.T_z;
}

void ForceTorque_s::set(double F_x, double F_y, double F_z, double T_x, double T_y, double T_z){
    this->F_x = F_x;
    this->F_y = F_y;
    this->F_z = F_z;
    this->T_x = T_x;
    this->T_y = T_y;
    this->T_z = T_z;
}

Vector3d ForceTorque_s::getForcesVector(){ return Vector3d(F_x, F_y, F_z); }
Vector3d ForceTorque_s::getTorquesVector(){ return Vector3d(T_x, T_y, T_z);}

VectorXd ForceTorque_s::getAsEigVector(){
    VectorXd vec(6);
    vec << F_x, F_y, F_z, T_x, T_y, T_z;
    return vec;
}