#include "FindNearestConfiguration.hpp"


double calcolaDistanza(const Eigen::VectorXd& config1, const Eigen::VectorXd& config2, const Eigen::VectorXd& pesi) {
    Eigen::VectorXd differenza = config1 - config2;
     return (differenza.cwiseProduct(pesi)).squaredNorm();
}

// Trova la configurazione target più vicina
int trovaIndiceConfigurazioneVicina(const Eigen::VectorXd& configurazioneRobot, const Eigen::MatrixXd& setConfigurazioniTarget, const Eigen::VectorXd& pesi) {
    double minDistanza = std::numeric_limits<double>::max();
    int indiceConfigurazioneVicina = -1;

    for (int i = 0; i < setConfigurazioniTarget.rows(); ++i) {
        const Eigen::VectorXd& target = setConfigurazioniTarget.row(i);
        double distanza = calcolaDistanza(configurazioneRobot, target, pesi);
        if (distanza < minDistanza) {
            minDistanza = distanza;
            indiceConfigurazioneVicina = i;
        }
    }

    return indiceConfigurazioneVicina;
}