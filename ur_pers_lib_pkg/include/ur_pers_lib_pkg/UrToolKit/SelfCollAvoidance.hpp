#pragma once

#include <eigen3/Eigen/Dense>
#include <vector>



class SelfCollAvoidance
{
private:
    Eigen::Vector3d computeCoulombForce(double q1, double q2, Eigen::Vector3d r_dist_vect);
    double computeSphereDistances() const;
    double k = 8.99e9;
    void calculateAllCoulombForces();
    int num_of_charges;
    Eigen::MatrixXd charges_matrix;
    Eigen::MatrixXd masses_matrix;
    Eigen::MatrixXd position_masses_matrix;
    Eigen::MatrixXd coulomb_forces_matrix;
    
public:
    SelfCollAvoidance(int num_of_carges);
    void addSphereParam(double mass, double charge, int index);
    void removeInteractions(int mass1, int mass2);
    void updateSpherePosition(int index, Eigen::Vector3d position);
    ~SelfCollAvoidance();
};

