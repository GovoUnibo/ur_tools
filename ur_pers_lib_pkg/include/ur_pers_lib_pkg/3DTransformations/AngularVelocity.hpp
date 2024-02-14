#pragma once


#include <eigen3/Eigen/Dense>
#include <math.h>


namespace AngVel{

    struct AngularVelocity_struct {
        double w_x, w_y, w_z;

        AngularVelocity_struct();
        AngularVelocity_struct(Eigen::Vector3d angularVelocity);
        ~AngularVelocity_struct();
        
        void operator=(const AngularVelocity_struct& omega);

    };

} // namespace AngVel