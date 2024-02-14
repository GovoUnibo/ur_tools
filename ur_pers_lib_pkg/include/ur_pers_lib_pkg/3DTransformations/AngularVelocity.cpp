#include "AngularVelocity.hpp"

using namespace Eigen;

namespace AngVel{

AngularVelocity_struct::AngularVelocity_struct()
:w_x(0)
,w_y(0)
,w_z(0)
{}

AngularVelocity_struct::AngularVelocity_struct(Vector3d angularVelocity)
{
  w_x = angularVelocity(0);
  w_y = angularVelocity(1);
  w_z = angularVelocity(2);
}

AngularVelocity_struct::~AngularVelocity_struct(){}

void AngularVelocity_struct::operator=(const AngularVelocity_struct& omega)
{
  w_x = omega.w_x;
  w_y = omega.w_y;
  w_z = omega.w_z;
}

}// namespace AngVel