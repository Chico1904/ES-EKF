#ifndef _CREATEMEAS
#define _CREATEMEAS
#include "structs.h"
#include <Eigen/Dense>
#include <armadillo>

pseudoLandmarks create_measurements(const dataset &my_dataset, // pseudoLandmarks
                        const Eigen::Matrix<double,7,7> pseudo_Q, 
                        const outSpecs outlier_specs, 
                        const int num_landmarks);
                        
arma::mat matrixxd_to_armamat(Eigen::MatrixXd eigen_A);

Eigen::VectorXd armamat_to_vectorxd(const arma::vec& arma_vec);

#endif