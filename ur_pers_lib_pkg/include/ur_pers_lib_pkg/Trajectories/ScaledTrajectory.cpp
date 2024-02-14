#include "ScaledTrajectory.hpp"

#include <ur_pers_lib_pkg/3DTransformations/Roll_Pitch_Yaw.hpp>
#include <ur_pers_lib_pkg/3DTransformations/AngularVelocity.hpp>

using namespace std;
using namespace Eigen;


namespace Traj{

    ScaledTrajectory::ScaledTrajectory()
    :traj5()
    ,traj_trap(0, 1, 0, 0, 0, 1)
    {}

    ScaledTrajectory::ScaledTrajectory(TrajParam traj_param, string traj_type)
    :traj5()
    ,traj_trap(0, 1, 0, 0, 0, 1)
    {
        if (traj_type == "poly5")
            this->setPoly5Param(traj_param.T, traj_param.time_discretizzation);
        else if (traj_type == "trapezoidal")
            this->setTrapezoidalParam(traj_param.acc_time, traj_param.dec_time, traj_param.traj_period, traj_param.time_discretizzation);
        else
            cout <<  "\033[1;31m" << "ERROR: Trajectory type: " + traj_type + " not implemented yet" << "\033[0m" << endl;            
    }

    ScaledTrajectory::ScaledTrajectory(string traj_type, double vel_max, double acc_time, double dec_time, double traj_period, double discretizzation)
    : traj5()
    , traj_trap(0, 1, 0, 0, 0, 1)
    {
        if (traj_type == "poly5")
            this->setPoly5Param(traj_period, discretizzation);
        else if (traj_type == "trapezoidal")
            this->setTrapezoidalParam(acc_time, dec_time, traj_period, discretizzation);
        else
            cout <<  "\033[1;31m" << "ERROR: Trajectory type: " + traj_type + " not implemented yet" << "\033[0m" << endl;
    }
    
    
    ScaledTrajectory::~ScaledTrajectory(){}



    void ScaledTrajectory::setPoly5Param(double T, double discretizzation){
        s.clear();
        s_dot.clear();
        s_dotdot.clear();
        traj5 = Poly5Trajectory(1, 0, 0, T);
        traj5.computeTrajectoryCoefficient();
        s = traj5.getTrajPositionPoints(discretizzation);
        s_dot = traj5.getTrajVelocityPoints(discretizzation);
        s_dotdot = traj5.getTrajAccelerationPoints(discretizzation);
    }

    void ScaledTrajectory::setTrapezoidalParam(double acc_time, double dec_time, double traj_period, double discretizzation){
            double qf_scaled_trajectory = 1;
            double qi_scaled_trajectory = 0;
            float max_velocity = (qf_scaled_trajectory-qi_scaled_trajectory)/(traj_period - (acc_time + dec_time)/2);
            traj_trap.setMaxVelocity(max_velocity);
            traj_trap.setAccTime(acc_time);
            traj_trap.setDecTime(dec_time);
            traj_trap.setTrajPeriod(traj_period);
            s.clear();
            s_dot.clear();
            s       = traj_trap.getDiscretizedPositionValues(discretizzation);
            s_dot   = traj_trap.getDiscretizedVelocityValues(discretizzation);
        
    }

    void ScaledTrajectory::setTrajectoryType(string traj_type, double traj_period, double discretizzation, double acc_time, double dec_time, double vel_max)
    {
        if (traj_type == "poly5")
            this->setPoly5Param(traj_period, discretizzation);
        else if (traj_type == "trapezoidal")
            this->setTrapezoidalParam(acc_time, dec_time, traj_period, discretizzation);
        else
            cout <<  "\033[1;31m" << "ERROR: Trajectory type: " + traj_type + " not implemented yet" << "\033[0m" << endl;
    }

    int ScaledTrajectory::getNumOfPosPoints() { return this->s.size(); }

    vector<double> ScaledTrajectory::getScaledPositionPoints(){    return this->s;}
    vector<double> ScaledTrajectory::getScaledVelocityPoints(){    return this->s_dot;}
    vector<double> ScaledTrajectory::getScaledAccelerationPoints(){    return this->s_dotdot;}

    vector<double> ScaledTrajectory::scalePosition(double p0, double pf){// formula O(s) = O_start + s(O_end - O_start)
        vector<double> p_traj(this->s.size());
        for (int i = 0; i < this->s.size(); i++)
            p_traj[i] = p0 + this->s[i]*(pf - p0);
        return p_traj;
    }
    vector<double> ScaledTrajectory::scaleVelocity(double p0, double pf){// formula O(s) = O_start + s(O_end - O_start)
        vector<double> v_traj(this->s_dot.size());
        for (int i = 0; i < this->s.size(); i++)
            v_traj[i] = s_dot[i]*(pf - p0);
        return v_traj;
    }

    std::vector<double> ScaledTrajectory::scaleAcceleration(double p0, double pf){
        std::vector<double> a_traj(this->s_dotdot.size());
        for (int i = 0; i < this->s.size(); i++)
            a_traj[i] = s_dotdot[i]*(pf - p0);
        return a_traj;
    }





} // namespace ScaledTraj