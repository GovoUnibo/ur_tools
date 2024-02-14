#include "SelfCollAvoidance.hpp"

using namespace Eigen;
SelfCollAvoidance::SelfCollAvoidance(int num_of_carges)
:num_of_charges(num_of_carges)
,charges_matrix(num_of_carges, num_of_carges)
,masses_matrix(num_of_carges, num_of_carges)
,position_masses_matrix(num_of_carges, 3)
,coulomb_forces_matrix(num_of_carges, 3) 
//righe = numero di cariche, colonne = 3 (x,y,z)
{
}

SelfCollAvoidance::~SelfCollAvoidance()
{
}

void SelfCollAvoidance::addSphereParam(double mass, double charge, int index){
    // voglio settare tutta la colonna in modo da ottenere una matrice simmetrica
    this->masses_matrix.col(index) = mass;
    this->charges_matrix.col(index) = charge;
}

void SelfCollAvoidance::removeInteractions(int mass1, int mass2){
    this->charges_matrix(mass1, mass2) = 0;
    this->charges_matrix(mass2, mass1) = 0;
}

void SelfCollAvoidance::updateSpherePosition(int index, Vector3d position){
    this->position_masses_matrix.row(index) = position;
}

Vector3d SelfCollAvoidance::computeCoulombForce(double q1, double q2, Vector3d r_dist_vect){ 
    double r = r_dist_vect.norm();
    return (k * std::abs(q1 * q2) / (r * r)) * r_dist_vect.normalized(); 
    
}

void SelfCollAvoidance::calculateAllCoulombForces() {

    for (int i = 0; i < num_of_charges; ++i) {
        Vector3d force = Vector3d::Zero();

        for (int j = 0; j < num_of_charges; ++j) {
            if (i != j) {
                Vector3d r_dist_vect = position_masses_matrix.row(i) - position_masses_matrix.row(j);
                double q1 = charges_matrix(i, j);
                double q2 = charges_matrix(j, i);
                double r = r_dist_vect.norm();

                Vector3d r_unit = r_dist_vect.normalized();
                Vector3d partial_force = (k * std::abs(q1 * q2) / (r * r)) * r_unit;

                force += partial_force;
            }
        }

    // Ora `force` contiene la forza totale sulla particella `i`
    }

}