#pragma once



#include <math.h>
#include <vector>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <string>

#include "Poly5Trajectory.hpp"
#include "TrapezoidalTrajectory.hpp"




namespace Traj{

    struct TrajParam{
        double T, time_discretizzation;
        double vel_max, acc_time, dec_time, traj_period;
    };    

    class ScaledTrajectory{

        private:
            
            std::vector<double> s, s_dot, s_dotdot;

            Poly5Trajectory traj5;
            TrapezoidalTrajectory traj_trap;
            double T, time_discretizzation;
            


            void setPoly5Param(double T, double discretizzation);
            void setTrapezoidalParam(double acc_time=0, double dec_time=0, double traj_period=1, double discretizzation=1);
            

        public:
            ScaledTrajectory();
            ScaledTrajectory(TrajParam traj_param, std::string traj_type);
            ScaledTrajectory(std::string traj_type, double vel_max, double acc_time, double dec_time, double traj_period, double discretizzation);
            ~ScaledTrajectory();

            

            void setTrajectoryType(std::string traj_type, double traj_period, double discretizzation, double acc_time=1, double dec_time=1, double vel_max=1);
            int getNumOfPosPoints();

            std::vector<double> getScaledPositionPoints();
            std::vector<double> getScaledVelocityPoints();
            std::vector<double> getScaledAccelerationPoints();

            std::vector<double> scalePosition(double p0, double pf); // formula O(s) = O_start + s(O_end - O_start) given a scaled trajectory s compute the cartesian position trajectory
            std::vector<double> scaleVelocity(double p0, double pf); // formula O(s) = O_start + s(O_end - O_start) given a scaled trajectory s compute the cartesian velocity trajectory
            std::vector<double> scaleAcceleration(double p0, double pf); // formula O(s) = O_start + s(O_end - O_start) given a scaled trajectory s compute the cartesian acceleration trajectory


            
    };
    

    


}


