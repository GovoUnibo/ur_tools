#include "Quaternion_Operations.hpp"




using namespace Eigen;
using namespace AngVel;
using namespace RPY;
using namespace MyQuaternion;




RPY_Angle_s Quaternion_To_RPY(const Quaternion_s q)
{
    RPY_Angle_s angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1)
        angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}

Quaternion_s RPY_To_Quaternion(double roll, double pitch, double yaw) // yaw (Z), pitch (Y), roll (X)
{
    // Abbreviations for the various angular functions
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    Quaternion_s q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}

Quaternion_s quaternion_negation(const Quaternion_s v) {
  Quaternion_s res;
  res.w = -v.w;
  res.x = -v.x;
  res.y = -v.y;
  res.z = -v.z;

  return res;
}

Quaternion_s quaternion_conjugate(const Quaternion_s q) {  
  Quaternion_s q_star;
  q_star.w =  q.w;
  q_star.x = -q.x;
  q_star.y = -q.y;
  q_star.z = -q.z;

  return q_star;
}

Quaternion_s quaternion_scalar_product(Quaternion_s v, double t) {

  Quaternion_s res;
  res.w = t * v.w;
  res.x = t * v.x;
  res.y = t * v.y;
  res.z = t * v.z;

  return res;
}

Quaternion_s quaternion_product(Quaternion_s q1, Quaternion_s q2) {

  Quaternion_s res;

  res.w = (q1.w*q2.w) - (q1.x*q2.x) - (q1.y*q2.y) - (q1.z*q2.z);
  res.x = (q1.w*q2.x) + (q1.x*q2.w) + (q1.y*q2.z) - (q1.z*q2.y);
  res.y = (q1.w*q2.y) + (q1.y*q2.w) + (q1.z*q2.x) - (q1.x*q2.z);
  res.z = (q1.w*q2.z) + (q1.z*q2.w) + (q1.x*q2.y) - (q1.y*q2.x);

  return res;
}

Quaternion_s quaternion_minus(const Quaternion_s v1, const Quaternion_s v0) 
{

  Quaternion_s vdiff;
  vdiff.w = v1.w - v0.w;
  vdiff.x = v1.x - v0.x;
  vdiff.y = v1.y - v0.y;
  vdiff.z = v1.z - v0.z;

  return vdiff;
}

Quaternion_s quaternion_plus(const Quaternion_s v1, const Quaternion_s v0) 
{
  Quaternion_s vadd;
  vadd.w = v1.w + v0.w;
  vadd.x = v1.x + v0.x;
  vadd.y = v1.y + v0.y;
  vadd.z = v1.z + v0.z;

  return vadd;
}

double quaternion_norm(const Quaternion_s q) 
{
  
  double q_norm;

  q_norm = sqrt( pow(q.w, 2.0) + pow(q.x, 2.0) + pow(q.y, 2.0) + pow(q.z, 2.0) );


  return q_norm;
}

Quaternion_s quaternion_normalize(Quaternion_s q)
{

  double d = quaternion_norm(q);

  q.w = q.w/d;
  q.x = q.x/d;
  q.y = q.y/d;
  q.z = q.z/d;

  return q;
}

Quaternion_s rotation_composition(Quaternion_s q_01, const Quaternion_s q_02){
//this is valid only if we are using unit quaternion
    Quaternion_s diff =  q_01 *quaternion_conjugate(q_02);

    return diff;
}


double quaternion_theta(const Quaternion_s q){
  
  double cos_theta =  q.w;

  return cos_theta;
}

double dot_product(Quaternion_s q_0 , Quaternion_s q_1){
    return q_0.w*q_1.w + q_0.x*q_1.x + q_0.y*q_1.y + q_0.z*q_1.z;
}

//FROM EULER mATRIX
//inline: direttiva del compilatore per l'espansione del corpo della funzione, nel punto in cui è chiamata

// Con define, espansione di preprocessore --> Usare #define è un metodo alternativo di inline
//#define NORM_M(float a, float b, float c, float d)		sqrt((a) * (a) + (b) * (b) + (c) * (c) + (d) * (d))

/*// Esempio di chiamata della macro che calcola la norma
// con macro
float r1 = NORM_M(q0, q1, q2, q3);
// Dopo espansione di preprocessore r1 diventa:
float r1 = sqrt((q0) * (q0) + (q1) * (q1) + (q2) * (q2) + (q3) * (q3))*/
inline float SIGN(float x) { 
  return (x >= 0.0f) ? +1.0f : -1.0f; 
}
inline float NORM(float a, float b, float c, float d) { 
  return sqrt(a * a + b * b + c * c + d * d); 
}

Quaternion_s rotMatrix_toQuaternion(const Matrix3d & rotMatrix) {

  double r11 = rotMatrix(0, 0);
  double r12 = rotMatrix(0, 1);
  double r13 = rotMatrix(0, 2);
  double r21 = rotMatrix(1, 0);
  double r22 = rotMatrix(1, 1);
  double r23 = rotMatrix(1, 2);
  double r31 = rotMatrix(2, 0);
  double r32 = rotMatrix(2, 1);
  double r33 = rotMatrix(2, 2);
  double q0 = (r11 + r22 + r33 + 1.0) / 4.0;
  double q1 = (r11 - r22 - r33 + 1.0) / 4.0;
  double q2 = (-r11 + r22 - r33 + 1.0) / 4.0;
  double q3 = (-r11 - r22 + r33 + 1.0) / 4.0;
  if (q0 < 0.0f) {
    q0 = 0.0;
  }
  if (q1 < 0.0f) {
    q1 = 0.0;
  }
  if (q2 < 0.0f) {
    q2 = 0.0;
  }
  if (q3 < 0.0f) {
    q3 = 0.0;
  }
  q0 = sqrt(q0);
  q1 = sqrt(q1);
  q2 = sqrt(q2);
  q3 = sqrt(q3);
  if (q0 >= q1 && q0 >= q2 && q0 >= q3) {
    q0 *= +1.0;
    q1 *= SIGN(r32 - r23);
    q2 *= SIGN(r13 - r31);
    q3 *= SIGN(r21 - r12);
  }
  else if (q1 >= q0 && q1 >= q2 && q1 >= q3) {
    q0 *= SIGN(r32 - r23);
    q1 *= +1.0f;
    q2 *= SIGN(r21 + r12);
    q3 *= SIGN(r13 + r31);
  }
  else if (q2 >= q0 && q2 >= q1 && q2 >= q3) {
    q0 *= SIGN(r13 - r31);
    q1 *= SIGN(r21 + r12);
    q2 *= +1.0;
    q3 *= SIGN(r32 + r23);
  }
  else if (q3 >= q0 && q3 >= q1 && q3 >= q2) {
    q0 *= SIGN(r21 - r12);
    q1 *= SIGN(r31 + r13);
    q2 *= SIGN(r32 + r23);
    q3 *= +1.0;
  }
  else {
    printf("coding error\n");
  }
  double r = NORM(q0, q1, q2, q3);
  q0 /= r;
  q1 /= r;
  q2 /= r;
  q3 /= r;

  Quaternion_s result ;
  result.w = q0;
  result.x = q1;
  result.y = q2;
  result.z = q3;
  return result;
}

Quaternion_s slerp(Quaternion_s quat_a, Quaternion_s quat_b, double t){
  // Only unit Quaternion_ss are valid rotations.

  quat_a = quaternion_normalize(quat_a);
  quat_b = quaternion_normalize(quat_b);


  double dot = dot_product(quat_a, quat_b); //cosHalfTheta


  if (dot < 0.0f) {    // If the dot product is negative, slerp won't take the shorter path. Note that v1 and -v1 are equivalent when
    // the negation is applied to all four components. Fix by reversing one Quaternion_s.
    quat_b = quaternion_negation(quat_b);
    dot = -dot;
  }
  
  const double DOT_THRESHOLD = 0.9995;         // If the inputs are too close for comfort, linearly interpolate and normalize the result.
  if (dot > DOT_THRESHOLD) {
    Quaternion_s result = quat_a + t*(quat_b - quat_a);
    result = quaternion_normalize(result);
    return result;
  }

  // Since dot is in range [0, DOT_THRESHOLD], acos is safe
  double theta_0      = acos(dot);        // theta_0 = angle between input vectors
  double theta        = theta_0*t;          // theta = angle between v0 and result
  double sin_theta    = sin(theta);     // compute this value only once
  double sin_theta_0  = sin(theta_0); // compute this value only once

  double s0 = cos(theta) - dot * sin_theta / sin_theta_0;  // == sin(theta_0 - theta) / sin(theta_0)
  double s1 = sin_theta / sin_theta_0;

  return (s0 * quat_a) + (s1 * quat_b);
}


AngularVelocity_struct rotationVelocity(Quaternion_s quat_actual, Quaternion_s quat_obj, double T_period) //quat_a = actual-- quat_b = objective
{
  Eigen::MatrixXd E;
  
  E.resize(3,4);
  //sampling time è quanto voglio la rotazione ci impieghi
  //quat_actual = quaternion_normalize(quat_actual);
  //quat_obj = quaternion_normalize(quat_obj);

  double dot = dot_product(quat_actual, quat_obj);

  if (dot < 0.0f) {    // If the dot product is negative, slerp won't take the shorter path. Note that v1 and -v1 are equivalent when
    // the negation is applied to all four components. Fix by reversing one Quaternion_s.
    quat_obj = quaternion_negation(quat_obj);
    dot = -dot;
  }
  Vector4d actual, obj;
  actual << quat_actual.w, quat_actual.x, quat_actual.y, quat_actual.z;
  obj << quat_obj.w, quat_obj.x, quat_obj.y, quat_obj.z;

  E <<  -quat_actual.x, quat_actual.w, -quat_actual.z, quat_actual.y, 
        -quat_actual.y, quat_actual.z, quat_actual.w, -quat_actual.x,
        -quat_actual.z, -quat_actual.y, quat_actual.x, quat_actual.w;

/*  E << -quat(1, 0), quat(0, 0), -quat(3, 0), quat(2, 0), -quat(2, 0),
      quat(3, 0), quat(0, 0), -quat(1, 0), -quat(3, 0), -quat(2, 0), quat(1, 0),
      quat(0, 0);
    Vector3d w = 2 * cycleHz * E * (obj - actual);*/

  Vector3d w = 2 * 1/T_period * E * (obj - actual);


  AngularVelocity_struct angularVelocity(w);

  return angularVelocity;
  
  
}




//TROVATO ONLINE
/*Quaternion_s slerp(Quaternion_s qa, Quaternion_s qb, double t) {
  // quaternion to return
  Quaternion_s qm;
  // Calculate angle between them.
  double cosHalfTheta = qa.w * qb.w + qa.x * qb.x + qa.y * qb.y + qa.z * qb.z;
  // if qa=qb or qa=-qb then theta = 0 and we can return qa
  if (abs(cosHalfTheta) >= 1.0){
    qm.w = qa.w;qm.x = qa.x;qm.y = qa.y;qm.z = qa.z;
    return qm;
  }
  // Calculate temporary values.
  double halfTheta = acos(cosHalfTheta);
  double sinHalfTheta = sqrt(1.0 - cosHalfTheta*cosHalfTheta);
  // if theta = 180 degrees then result is not fully defined
  // we could rotate around any axis normal to qa or qb
  if (fabs(sinHalfTheta) < 0.001){ // fabs is floating point absolute
    qm.w = (qa.w * 0.5 + qb.w * 0.5);
    qm.x = (qa.x * 0.5 + qb.x * 0.5);
    qm.y = (qa.y * 0.5 + qb.y * 0.5);
    qm.z = (qa.z * 0.5 + qb.z * 0.5);
    return qm;
  }
  double ratioA = sin((1 - t) * halfTheta) / sinHalfTheta;
  double ratioB = sin(t * halfTheta) / sinHalfTheta; 
  //calculate Quaternion.
  qm.w = (qa.w * ratioA + qb.w * ratioB);
  qm.x = (qa.x * ratioA + qb.x * ratioB);
  qm.y = (qa.y * ratioA + qb.y * ratioB);
  qm.z = (qa.z * ratioA + qb.z * ratioB);
  return qm;
}*/

