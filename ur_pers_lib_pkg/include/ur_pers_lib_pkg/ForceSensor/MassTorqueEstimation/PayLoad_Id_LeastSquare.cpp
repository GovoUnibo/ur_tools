#include "PayLoad_Id_LeastSquare.hpp"

using namespace std;
using namespace Eigen;
using namespace RPY;

PayLoad_LeastSquare::PayLoad_LeastSquare(ros::NodeHandle& nh, ros::Rate sensor_update_rate_hz, string topic_name) 
: SensorReader(nh, sensor_update_rate_hz, topic_name)
, ftEstimation(6)
{
    mass = 0.0;
    Tx0 = Ty0 = Tz0 = 0.0;
    cgX = cgY = cgZ = 0.0; // Initialize center of gravity
    k1 = k2 = k3 = 0.0; // Initialize k vector


    SensorReader::setUseMovingAverageFilter(true);


}

PayLoad_LeastSquare::~PayLoad_LeastSquare() {}

void PayLoad_LeastSquare::computeBiasUsingGripperMass(double gripper_mass){
    // this->mass = gripper_mass;
    //compute bias
    SensorReader::setSensorBias(gripper_mass);
    VectorXd bias = SensorReader::getSensorBias();
    this->Fx0 = bias(0); this->Fy0 = bias(1); this->Fz0 = bias(2); this->Tx0 = bias(3); this->Ty0 = bias(4); this->Tz0 = bias(5);
    SensorReader::resetMass_and_CenterOfMass();

}

void PayLoad_LeastSquare::computeBiasUsing2RobotPos(){
    SensorReader::setSensorBias_Using2RobotPos(); // leggi le istruzioni da terminale
    VectorXd bias = SensorReader::getSensorBias();
    this->Fx0 = bias(0); this->Fy0 = bias(1); this->Fz0 = bias(2); this->Tx0 = bias(3); this->Ty0 = bias(4); this->Tz0 = bias(5);
}

void PayLoad_LeastSquare::setBias(double fx0, double fy0, double fz0, double tx0, double ty0, double tz0){
    SensorReader::setSensorBias(fx0, fy0, fz0, tx0, ty0, tz0);
    this->Fx0 = fx0; this->Fy0 = fy0; this->Fz0 = fz0; this->Tx0 = tx0; this->Ty0 = ty0; this->Tz0 = tz0;
}


void PayLoad_LeastSquare::registerFTValues(Eigen::Vector3d eeRot) {
    SensorReader::setUseCompensateGravity(false);

    VectorXd ft = SensorReader::getWrenchInSensorFrame();

    //write in red forces registered
    cout << "\033[1;31m" << "Forces registered: " << ft(0) << " " << ft(1) << " " << ft(2) <<  endl;
    cout << "Torques registered: " << ft(3) << " " << ft(4) << " " << ft(5) << endl;
    cout << "Rotations registered: " << eeRot(0) << " " << eeRot(1) << " " << eeRot(2) << "\033[0m" << endl;
    double fx = ft(0); double fy = ft(1); double fz = ft(2); double tx = ft(3); double ty = ft(4); double tz = ft(5);
    MatrixXd combinedMatrix(3, 6); // skewsym matrix + identity matrix
    Vector3d torqueVector;
    combinedMatrix << 0, -fz, fy, 1, 0, 0,
                      fz, 0, -fx, 0, 1, 0,
                      -fy, fx, 0, 0, 0, 1;
    
    torqueVector << tx, ty, tz;

    massEstimationForces.push_back(Vector3d(fx, fy, fz));
    //porta il vettore accelerazione nel sensor frame g_sf = R^T * g
    gravityRotatedVector.push_back(RPY_To_RotationMatrix(eeRot(0), eeRot(1), eeRot(2)).transpose()*gravityVector); 

    this->Torques.push_back(torqueVector);
    this->Forces.push_back(combinedMatrix);
}

void PayLoad_LeastSquare::estimationOfMass(){
    VectorXd F_MassEstimation(3 * massEstimationForces.size());
    VectorXd G(3 * gravityRotatedVector.size());
    
    for (size_t i = 0; i < massEstimationForces.size(); i++) 
        F_MassEstimation.segment(3 * i, 3) = massEstimationForces[i];

    for (size_t i = 0; i < gravityRotatedVector.size(); i++) 
        G.segment(3 * i, 3) = gravityRotatedVector[i];
    
    double numerator = G.transpose() * F_MassEstimation;
    double denominator = G.transpose() * G;
    
    this->mass = numerator / denominator;

    cout << "mass: " << mass << endl;
}

void PayLoad_LeastSquare::CoG_Identification(){
    if (mass == 0.0)
        this->estimationOfMass();
    // se la size di Torques e Force è < di 3 allora alza errore
    if (Torques.size() < 3 || Forces.size() < 3)
        throw std::runtime_error("Not enough data to identify the center of gravity");
    
    VectorXd tau;
    MatrixXd F;

    tau.conservativeResize(3 * Torques.size());
    size_t index = 0;
    for (const Eigen::Vector3d& torque : Torques) {
        tau.segment(index, 3) = torque;
        index += 3;
    }
    
    F.conservativeResize(3 * Forces.size(), 6);
    index = 0;
    for (const Eigen::MatrixXd& force : Forces) {
        F.block(index, 0, 3, 6) = force;
        index += 3;
    }
    
    Eigen::VectorXd p = (F.transpose() * F).inverse() * F.transpose() * tau;

    this->cgX = p(0);
    this->cgY = p(1);
    this->cgZ = p(2);

    // SensorReader::addMass_and_CenterOfMass(this->mass, this->cgX, this->cgY, this->cgZ);


    // scrivi in verde il centro di gravità e massa
    cout << "\033[1;32m" << "Mass: " << mass << "\033[0m" << endl;
    cout << "\033[1;32m" << "Center of gravity: " << cgX << " " << cgY << " " << cgZ << "\033[0m" << endl;

    k1 = p(3);
    k2 = p(4);
    k3 = p(5);

}



Vector3d PayLoad_LeastSquare::getTorqueEstimationVector(double fx, double fy, double fz){
    
    double T_x = fy * cgZ - fz * cgY + k1;
    double T_y = fz * cgX - fx * cgZ + k2;
    double T_z = fx * cgY - fy * cgX + k3;

    return Vector3d(T_x, T_y, T_z);
}

VectorXd PayLoad_LeastSquare::getFTVectorGripperRejected(double rx, double ry, double rz, bool in_base_frame){
    R_sb = RPY_To_RotationMatrix(rx, ry, rz);
    VectorXd ft = SensorReader::getWrenchInSensorFrame(); // unbiased wrench



    ftEstimation << this->mass * (R_sb.transpose() * gravityVector), getTorqueEstimationVector(ft(0), ft(1), ft(2));

    // cout << "Force Gravity: " << ftEstimation.head(3).transpose() << endl;
    // cout << "Force Letta " << ft.head(3).transpose() << endl;
    VectorXd ft_compensated = ft - ftEstimation;

    if (in_base_frame)
        return R_sb * ft_compensated;
    else
        return ft_compensated;
    
}
