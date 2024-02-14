#include "EeRots_for_MassEstimation.hpp"

using namespace Eigen;
using namespace std;

ListOfEndEffectorRotation::ListOfEndEffectorRotation()
: eeRot(24)
{


        eeRot[0] << 1,  0,  0,
                    0, -1,  0,
                    0,  0, -1;

        eeRot[1] << 1, 0, 0,
                    0, 1, 0,
                    0, 0, 1;

        eeRot[2] << 1, 0,  0,
                    0, 0, -1,
                    0, 1,  0;

        eeRot[3] << 1,  0, 0,
                    0,  0, 1,
                    0, -1, 0;

        eeRot[4] << -1, 0,  0,
                     0, 1,  0,
                     0, 0, -1;

        eeRot[5] << -1,  0, 0,
                     0, -1, 0,
                     0,  0, 1;

        eeRot[6] << -1, 0, 0,
                     0, 0, 1,
                     0, 1, 0;

        eeRot[7] << -1,  0,  0,
                     0,  0, -1,
                     0, -1,  0;
        
        eeRot[8] <<  0, 1, 0,
                     1, 0, 0,
                     0, 0, -1;

        eeRot[9] <<  0, -1, 0,
                     1,  0, 0,
                     0,  0, 1;
        
        eeRot[10] <<  0, 0, 1,
                      1, 0, 0,
                      0, 1, 0;
        
        eeRot[11] <<  0, 0, -1,
                      1, 0,  0,
                      0, -1, 0;
        
        eeRot[12] <<  0, 1,  0,
                      -1, 0, 0,
                      0, 0,  1;

        eeRot[13] <<   0, -1,  0,
                      -1,  0,  0,
                       0,  0, -1;

        eeRot[14] <<  0,  0, -1,
                      -1, 0,  0,
                      0,  1,  0;
        
        eeRot[15] <<   0,  0, 1,
                      -1,  0, 0,
                       0, -1, 0;
        
        eeRot[16] <<  0, 1, 0,
                      0, 0, 1,
                      1, 0, 0;
        
        eeRot[17] <<  0, -1,  0,
                      0,  0, -1,
                      1,  0,  0;

        eeRot[18] <<  0,  0, -1,
                      0,  1,  0,
                      1,  0,  0;
        
        eeRot[19] <<  0,  0, 1,
                      0, -1, 0,
                      1,  0, 0;

        eeRot[20] <<  0,  1,  0,
                      0,  0, -1,
                      -1, 0,  0;
        
        eeRot[21] <<  0, -1, 0,
                      0,  0, 1,
                      -1, 0, 0;

        eeRot[22] <<   0,  0, 1,
                       0,  1, 0,
                      -1,  0, 0;

        eeRot[23] <<   0,  0, -1,
                       0, -1,  0,
                      -1,  0,  0;

}

ListOfEndEffectorRotation::~ListOfEndEffectorRotation(){}

void ListOfEndEffectorRotation::removeNonComplanarRotations(){

    std::vector<Eigen::Matrix3d> nonComplanarRotations;

    // Ciclo attraverso le matrici di rotazione e verifica se sono complanari
    for (int i = 0; i < eeRot.size(); ++i) {
        bool complanar = false;

        // Controlla la complanarità con tutte le matrici precedenti
        for (int j = 0; j < i; ++j) {
            if (areVectorsLinearlyDependent(eeRot[i].row(0), eeRot[j].row(0)) &&
                areVectorsLinearlyDependent(eeRot[i].row(1), eeRot[j].row(1)) &&
                areVectorsLinearlyDependent(eeRot[i].row(2), eeRot[j].row(2))) {
                complanar = true;
                break; // Se è complanare con una qualsiasi delle precedenti, la scartiamo
            }
        }

        if (!complanar) // Questa matrice di rotazione non è complanare con le precedenti, la conserviamo
            nonComplanarRotations.push_back(eeRot[i]);
    }
    this->eeRot = nonComplanarRotations;
}


Eigen::Matrix3d ListOfEndEffectorRotation::getMatrix(int i) const {
    return eeRot[i];
}


Eigen::Matrix3d ListOfEndEffectorRotation::operator[](int i) const {
    return eeRot[i];
}

int ListOfEndEffectorRotation::size() const {
    return eeRot.size();
}

vector<Matrix3d> ListOfEndEffectorRotation::getList() const {
    return eeRot;
}


