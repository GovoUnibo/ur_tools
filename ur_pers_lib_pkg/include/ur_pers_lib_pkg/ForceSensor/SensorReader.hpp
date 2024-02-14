#pragma once

#include <ros/ros.h>
#include "geometry_msgs/WrenchStamped.h"
#include <iostream>
#include <ur_pers_lib_pkg/Filters/Filters.hpp>
#include <eigen3/Eigen/Dense>
#include <ur_pers_lib_pkg/3DTransformations/Roll_Pitch_Yaw.hpp>

struct ForceTorque_s
{
    double F_x;
    double F_y;
    double F_z;
    double T_x;
    double T_y;
    double T_z;
    ForceTorque_s(); 
    ForceTorque_s(double F_x, double F_y, double F_z, double T_x, double T_y, double T_z);
    ~ForceTorque_s();
    // operator overloading
    void operator=(const ForceTorque_s& ft);
    void operator=(const geometry_msgs::WrenchStamped& msg);
    void operator=(const Eigen::VectorXd& vec);

    void operator+=(const ForceTorque_s& ft);
    void operator-=(const ForceTorque_s& ft);

    void set(double F_x, double F_y, double F_z, double T_x, double T_y, double T_z);
    Eigen::Vector3d getForcesVector();
    Eigen::Vector3d getTorquesVector();
    Eigen::VectorXd getAsEigVector();
};

ForceTorque_s operator+(const ForceTorque_s& ft, const ForceTorque_s& ft2);
ForceTorque_s operator-(const ForceTorque_s& ft, const ForceTorque_s& ft2);

class SensorReader
{
public:

    SensorReader(ros::NodeHandle& nh, ros::Rate sensor_update_rate_hz, std::string topic_name);
    ~SensorReader();

    void FT_sensor_Reading_Callback(const geometry_msgs::WrenchStamped& msg);

    ForceTorque_s compensateGravity(); // prende in input l'orientazione del sensore rispetto alla base/mondo
    void addMass_and_CenterOfMass(double mass, double pos_x, double pos_y, double pos_z); // la trovi su urdf
    void setRotMatrix_sb(double rot_ft_x, double rot_ft_y, double rot_ft_z);

    void setUseLowPassFilter(bool useLowPassFilter);
    void setUseMovingAverageFilter(bool useMovingAverageFilter);
    void setUseCompensateGravity(bool useCompensateGravity);

    Eigen::VectorXd getWrenchInSensorFrame();
    Eigen::VectorXd getWrenchInBaseFrame(double rot_ft_x=0, double rot_ft_y=0, double rot_ft_z=0);

    void publishFT();

private:

    const ForceTorque_s _LowPassFilter(const ForceTorque_s msg);
    const ForceTorque_s _MovingAverageFilter(const ForceTorque_s& msg);
    bool useLowPassFilter = false;
    bool useMovingAverageFilter = false;
    bool useCompensateGravity = false;


    Eigen::Matrix3d position_matrix_wrt_sensor;
    Eigen::Vector3d position_vector_wrt_sensor;
    Eigen::Vector3d weight_vector, tool_gravity, torque_gravity; //3x1
    std::vector<double> link_masses;
    std::vector<Eigen::Vector3d> vector_of_link_baricenter_position; // vettore di punti in cui viene applicata la forza peso sul corpo rispetto al sensore
    ros::Rate publish_rate; // guardare dentro lo xacro del sensore dove sta <updateRate>#####</updateRate> 
    ros::Subscriber ft_subscriber;
    ros::Publisher ft_publisher;


    LowPassFilter lowPassFilter_fx, lowPassFilter_fy, lowPassFilter_fz, lowPassFilter_tx, lowPassFilter_ty, lowPassFilter_tz;
    MovingAverageFilter moving_average_fx, moving_average_fy, moving_average_fz, moving_average_tx, moving_average_ty, moving_average_tz;
    ForceTorque_s FT_measured, FT_bias;

    Eigen::Matrix3d R_matrix;

    ros::AsyncSpinner spinner;

};

