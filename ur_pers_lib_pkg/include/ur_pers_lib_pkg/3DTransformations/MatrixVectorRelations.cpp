#include "MatrixVectorRelations.hpp"



bool areRotMatricesComplanar(const Eigen::Matrix3d& A, const Eigen::Matrix3d& B) {
    double epsilon = 1e-6; // Tolleranza per la precisione
    double trace1 = A.trace();
    double trace2 = B.trace();

    // Verifica se le tracce delle due matrici sono uguali
    bool tracesEqual = std::abs(trace1 - trace2) < epsilon;

    return tracesEqual;
}

bool areVectorsLinearlyDependent(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2) {
    double epsilon = 1e-6; // Piccolo valore per la tolleranza
    // Calcola il prodotto scalare tra i due vettori
    double dotProduct = v1.dot(v2);
    // Verifica se il prodotto scalare è vicino a zero (linearmente dipendenti)
    return std::abs(dotProduct) < epsilon;
}
