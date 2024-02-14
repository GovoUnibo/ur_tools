
#ifndef TrapezoidProfile
#define TrapezoidProfile

#include <math.h>
#include <iostream>
#include <vector>



class TrapezoidalTrajectory{

    public:
        TrapezoidalTrajectory();
        TrapezoidalTrajectory(float intial_point, float final_point, float vel_max, float acc_time, float dec_time, float traj_period);
        ~TrapezoidalTrajectory();

        void setPuntoIniziale(float);
        void setPuntoFinale(float);
        void setMaxVelocity(float);
        void setAccTime(float);
        void setDecTime(float);
        void setTrajPeriod(float);
        void computeTrajCoefficient();

        std::vector<double> getDiscretizedPositionValues(double discretization); 
        std::vector<double> getDiscretizedVelocityValues(double discretization);

    

    private:
        void accPhase();        //create the acceleration phase EQUATION param
        void constVelPhase();   //create the constant velocity phase EQUATION param
        void decPhase();        //create the decceleration velocity phase EQUATION param

        bool    is_initial_point_set, 
                is_final_point_set, 
                is_max_vel_set, 
                is_acc_time_set,
                is_dec_time_set,
                is_traj_period_set;


        double time;

        //variabili acc phase
        double t_a, t_f, t_dec; //t = [0 --> ta]
        double v_max; 
        double q_0, q_f; //q_0 = punto iniziale

        double a_0, a_1, a_2; //coeff
        //variabili constant vel phse
        double b_0, b_1; 
        //variabili dec phase
        double c_0, c_1, c_2;

        
};




#endif
