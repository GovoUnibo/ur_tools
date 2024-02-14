#include "TrapezoidalTrajectory.hpp"


using namespace std;

inline double round_3dec( double val )
{
    return round( val * 1000.0 ) / 1000.0;
}
inline double round_4dec( double val )
{
    return round( val * 10000.0 ) / 10000.0;
}

TrapezoidalTrajectory::TrapezoidalTrajectory()
: q_0(0)
, q_f(0)
, v_max(0)
, t_a(0)
,t_dec(0)
, t_f(0)
{
        is_initial_point_set= false; 
        is_final_point_set  = false;
        is_max_vel_set      = false;
        is_acc_time_set     = false;
        is_traj_period_set  = false;
        is_dec_time_set     = false;
}


TrapezoidalTrajectory::~TrapezoidalTrajectory()
{
}

TrapezoidalTrajectory::TrapezoidalTrajectory(float intial_point, float final_point, float vel_max, float acc_time, float dec_time, float traj_period)
: q_0(intial_point)
, q_f(final_point)
, v_max(vel_max)
, t_a(acc_time)
, t_f(traj_period)
, t_dec(dec_time)
{
        is_initial_point_set= true; 
        is_final_point_set  = true;
        is_max_vel_set      = true;
        is_acc_time_set     = true;
        is_traj_period_set  = true;
        is_dec_time_set     = true;
}

void TrapezoidalTrajectory::setPuntoIniziale(float initial_pont){q_0   = initial_pont;          is_initial_point_set= true;}
void TrapezoidalTrajectory::setPuntoFinale(float final_pont)    {q_f   = final_pont;            is_final_point_set  = true;}
void TrapezoidalTrajectory::setMaxVelocity(float max_vel)       {v_max = max_vel;               is_max_vel_set      = true;}
void TrapezoidalTrajectory::setAccTime(float acc_time)          {t_a   = acc_time;              is_acc_time_set     = true;}
void TrapezoidalTrajectory::setDecTime(float dec_time)          {t_dec = dec_time;              is_dec_time_set     = true;}
void TrapezoidalTrajectory::setTrajPeriod(float traj_duration)  {t_f   = traj_duration;         is_traj_period_set  = true;}

    void TrapezoidalTrajectory::accPhase()
    {
        a_0 = q_0;
        a_1 = 0;
        a_2 = v_max/(2*t_a);
        //cout << a_0 << a_2 << endl;
    }

    void TrapezoidalTrajectory::constVelPhase()
    {
        b_1 = v_max;
        b_0 = q_0 - v_max*(t_a/2);
    }

    void TrapezoidalTrajectory::decPhase()
    {
        c_0 = q_f - (v_max/2)*(pow(t_f, 2)/t_dec);
        //c_0 = q_f - (v_max/t_dec)*pow(t_f, 2) - (v_max/2*t_dec)**pow(t_f, 2)
        c_1 = v_max*(t_f/t_dec);
        c_2 = -v_max/(2*t_dec);
    }

void TrapezoidalTrajectory::computeTrajCoefficient()            
{
    if( is_initial_point_set && is_final_point_set && is_max_vel_set && is_acc_time_set && is_traj_period_set)
    {
    accPhase();
    constVelPhase();
    decPhase();
    }
    else
        cout << "1 or more Traj Param are not set" << endl;
} 

vector<double> TrapezoidalTrajectory::getDiscretizedPositionValues(double discrezation){
        
        double t=0;
        int i=0;
        vector<double> vettoreDeiPunti;
        computeTrajCoefficient();
        //cout << "Fase di Accelerazione\n" << endl;
        while(t < round_4dec(t_a)) 
        {
            vettoreDeiPunti.push_back(a_0 + a_1*t + a_2*pow(t,2));
            //cout << "t:" << t << " " << vettoreDeiPunti[i] << endl;
            i++;
            t += discrezation;      
        }
        //cout << "Fase Costante\n";
        while( t <= round_4dec(t_f-t_dec))
        {   
            vettoreDeiPunti.push_back(b_0 + b_1*(t));
            t += discrezation;            
            //cout << "t:" << t << " " << vettoreDeiPunti[i] << endl;
            i++;
        }
        
        //cout << "Fase di Deccelerazione\n" << endl;
        while(t < round_4dec(t_f)) 
        {
            
            vettoreDeiPunti.push_back(c_0 + c_1*t + c_2*pow(t,2));
            t += discrezation;
            //cout << "t:" << t << " " << vettoreDeiPunti[i] << endl;
            i++;
        }
        //cout << vettoreDeiPunti.size() <<endl;
        return vettoreDeiPunti;
}

vector<double> TrapezoidalTrajectory::getDiscretizedVelocityValues(double discretization){
        
        
        double t=0;
        int i=0;
        vector<double> vettorePuntiVel;
        computeTrajCoefficient();
        //cout << "Fase di Accelerazione\n" << endl;
        while(t < round_4dec(t_a)) 
        {
            vettorePuntiVel.push_back(a_1 + 2*a_2*t);
            i++;
            t += round_4dec(discretization);
            //cout << "t:" << t << " " <<vettorePuntiVel[i] << endl;          
        }
        //cout << t << endl;
        while(t < round_4dec(t_f-t_dec))
        {   
            vettorePuntiVel.push_back(b_1);
            //cout << "t:" << t << " " <<vettorePuntiVel[i] << endl;
            t += round_4dec(discretization);  
            i++;
        }
        //cout << t << endl;
        //cout << "Fase di Deccelerazione\n" << endl;
        while(t < t_f + 0.0001)
        {
            
            vettorePuntiVel.push_back(c_1 + 2*c_2*t);
            //cout << "t:" << t << " " <<vettorePuntiVel[i] << endl;
            t += round_4dec(discretization);
            i++;
        }
        // for(int i =0; i < vettorePuntiVel.size(); i++)
        //      cout << vettorePuntiVel[i] << endl;

        //cout << vettoreDeiPunti.size() << endl;

        return vettorePuntiVel;
}