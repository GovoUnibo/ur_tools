#include "Poly5Trajectory.hpp"

using namespace std;

Poly5Trajectory::Poly5Trajectory()
{
    a0 = a1 = a2 = a3 =  a4 =  a5 = 0;
    q_0 = q_f = 0;
    q_dot_0 = q_dot_f = 0;
    q_dot_max = 0;                
    q_dot_dot_0 = q_dot_dot_f = 0;
    q_dotdot_max = 0;
    T = 0;
}
Poly5Trajectory::Poly5Trajectory(double qf, double q_dot_f, double q_dot_dot_f, double T){
    this->q_0 = 0;
    this->q_f = qf;

    this->q_dot_0 = 0;
    this->q_dot_f = q_dot_f;

    this->q_dot_dot_0= 0;
    this->q_dot_dot_f = q_dot_dot_f;

    this->T = T;
}

Poly5Trajectory::Poly5Trajectory(double a0, double a1, double a2, double a3, double a4, double a5, double T){
    this->a0 = a0;
    this->a1 = a1;
    this->a2 = a2;
    this->a3 = a3;
    this->a4 = a4;
    this->a5 = a5;
    this->T = T;
}


Poly5Trajectory::~Poly5Trajectory(){}

Poly5Trajectory& Poly5Trajectory::operator=(const Poly5Trajectory& other){
    this->a0 = other.a0;
    this->a1 = other.a1;
    this->a2 = other.a2;
    this->a3 = other.a3;
    this->a4 = other.a4;
    this->a5 = other.a5;
    this->q_0 = other.q_0;
    this->q_f = other.q_f;
    this->q_dot_0 = other.q_dot_0;
    this->q_dot_f = other.q_dot_f;
    this->q_dot_max = other.q_dot_max;
    this->q_dot_dot_0 = other.q_dot_dot_0;
    this->q_dot_dot_f = other.q_dot_dot_f;
    this->q_dotdot_max = other.q_dotdot_max;
    this->T = other.T;


    return *this;
}


void Poly5Trajectory::setInitialPoint(float q_initial)      {q_0 = q_initial;}
void Poly5Trajectory::setFinalPoint(float q_final)          {q_f = q_final;}
void Poly5Trajectory::setInitialVelocity(float q_dot_0)     {this->q_dot_0 = q_dot_0;}
void Poly5Trajectory::setFinalVelocity(float q_dot_f)       {this->q_dot_f = q_dot_f;}
void Poly5Trajectory::setInitialAcceleration(float q_2dot_0){this->q_dot_dot_0 = q_2dot_0;}
void Poly5Trajectory::setFinalAcceleration(float q_2dot_f)  {this->q_dot_dot_f = q_2dot_f;}
void Poly5Trajectory::setPeriod(float traj_period)          {T = traj_period;}

// parametri per la messa in scala cinematica
void Poly5Trajectory::setMaxVelocity(float v_max){q_dot_max = v_max;}
void Poly5Trajectory::setMaxVelocity_UsingPeriodAndAmplitude(float traj_period, float amplitude){ q_dot_max = (15*amplitude)/(8*traj_period); }

void Poly5Trajectory::setMaxAcceleration(float acc_max)     { q_dotdot_max = acc_max; }
void Poly5Trajectory::setMaxAcceleration_UsingPeriodAndAmplitude(float traj_period, float amplitude)    {q_dotdot_max = (10*sqrt(3)*amplitude)/(3*pow(traj_period,2)); }

void Poly5Trajectory::setPeriod_UsingAmplitudeAndMaxVel(float vel_max, float amplitude){ T = (15*amplitude)/(8*vel_max) ;}
void Poly5Trajectory::setPeriod_UsingAmplitudeAndMaxAcc(float acc_max, float amplitude){ T = sqrt( (10*sqrt(3)*amplitude)/(3*acc_max)); }


void Poly5Trajectory::computeScaledTrajectoryCoefficient(){

        this->T = 1;
        this->a0 = 0;
        this->a1 = 0;
        this->a2 = 0;
        this->a3 = 10;
        this->a4 = -15;
        this->a5 = 6;
    

}

void Poly5Trajectory::computeTrajectoryCoefficient(){

    double delta_q = q_f - q_0;
    double delta_q_dot_dot = q_dot_dot_f - q_dot_dot_0;

    this->a0 = q_0;
    this->a1 = q_dot_0;
    this->a2 = 0.5*q_dot_dot_0;
    this->a3 = (1/(2*pow(T,3)))*(20*delta_q - (8*q_dot_f + 12*q_dot_0)*T + (3*q_dot_dot_f - q_dot_dot_0)* pow(T,2) );
    this->a4 = (1/(2*pow(T,4)))*(-30*delta_q + (14*q_dot_f + 16*q_dot_0)*T + (3*q_dot_dot_f - 2*q_dot_dot_0)*pow(T,2));
    this->a5 = (1/(2*pow(T,5)))*(12*delta_q - 6*(q_dot_f + q_dot_0)*T - delta_q_dot_dot*pow(T,2));

    // this->printCoeff();
}

void Poly5Trajectory::printCoeff(){
        cout << "Scaled Trajectory Coefficients: " << endl;
        cout << "a0: " << this->a0 << endl;
        cout << "a1: " << this->a1 << endl;
        cout << "a2: " << this->a2 << endl;
        cout << "a3: " << this->a3 << endl;
        cout << "a4: " << this->a4 << endl;
        cout << "a5: " << this->a5 << endl;
}

vector<double> Poly5Trajectory::getTrajPositionPoints(double time_discretization)
{
    vector<double> discretizedTrajectory;
    int num_points = (int)(T/time_discretization);
    double t=0;
    for (int i = 0; i < num_points; i++){
        discretizedTrajectory.push_back(a0 + a1*t + a2*pow(t,2) + a3*pow(t,3) + a4*pow(t,4) + a5*pow(t,5));
        // cout << "t: " << t << "  pos: " << discretizedTrajectory[i] << endl;  
        t += time_discretization;
    }
    return discretizedTrajectory;
}

vector<double> Poly5Trajectory::getTrajVelocityPoints(double time_discretization)
{

    vector<double> discretizedTrajectory;

    int num_points = (int)(T/time_discretization);

    double t=0;
    for (int i = 0; i < num_points; i++){
        discretizedTrajectory.push_back(a1 + 2*a2*t + 3*a3*pow(t,2) + 4*a4*pow(t,3) + 5*a5*pow(t,4));
        t += time_discretization;
    }
    return discretizedTrajectory;
}


vector<double> Poly5Trajectory::getTrajAccelerationPoints(double time_discretization){
    
    vector<double> discretizedTrajectory;

    int num_points = (int)(T/time_discretization);

    double t=0;
    for (int i = 0; i < num_points; i++){
        //0.0001 è per risolvere l'arrotondamento dovuto al float 
        discretizedTrajectory.push_back(2*a2 + 6*a3*t + 12*a4*pow(t,2) + 20*a5*pow(t,3));
        t += time_discretization;
    }
    return discretizedTrajectory;
}
