#pragma once

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <limits>

double calcolaDistanza(const Eigen::VectorXd& config1, const Eigen::VectorXd& config2, const Eigen::VectorXd& pesi);
// Trova la configurazione target più vicina
int trovaIndiceConfigurazioneVicina(const Eigen::VectorXd& configurazioneRobot, const Eigen::MatrixXd& setConfigurazioniTarget, const Eigen::VectorXd& pesi);