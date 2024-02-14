#pragma once

#include <math.h>
#include <vector>
#include <iostream>

class Poly5Trajectory
{
    public:
        Poly5Trajectory();
        Poly5Trajectory(double qf, double q_dot_f, double q_dot_dot_f, double T);
        Poly5Trajectory(double a0, double a1, double a2, double a3, double a4, double a5, double T);
        ~Poly5Trajectory();

        void setInitialPoint(float);
        void setFinalPoint(float);
        void setPeriod(float);
        void setInitialVelocity(float);
        void setFinalVelocity(float);
        void setInitialAcceleration(float);
        void setFinalAcceleration(float);
        void printCoeff();

        Poly5Trajectory& operator=(const Poly5Trajectory& other);


        // parametri per la messa in scala cinematica
        void setMaxVelocity(float);
        void setMaxVelocity_UsingPeriodAndAmplitude(float period, float amplitude);
        void setMaxAcceleration(float);
        void setMaxAcceleration_UsingPeriodAndAmplitude(float period, float amplitude);
        void setPeriod_UsingAmplitudeAndMaxVel(float period, float max_vel);
        void setPeriod_UsingAmplitudeAndMaxAcc(float period, float max_acc);

        void computeScaledTrajectoryCoefficient();
        void computeTrajectoryCoefficient();

        std::vector<double> getTrajPositionPoints(double time_discretization);
        std::vector<double> getTrajVelocityPoints(double time_discretization);
        std::vector<double> getTrajAccelerationPoints(double time_discretization);


    private:
        
        double a0, a1, a2, a3, a4, a5;                  //coeff
        double q_0, q_f;                                //pos bound condition
        double q_dot_0, q_dot_f, q_dot_max;             // vel bound condition
        double q_dot_dot_0, q_dot_dot_f , q_dotdot_max; // acc bound condition

        double T;

};