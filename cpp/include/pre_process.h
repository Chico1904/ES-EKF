#ifndef _PP
#define _PP

#include <iostream>
#include <cmath>         
#include <vector>
#include <fstream>
#include <filesystem>
#include <ctime>

#include <Eigen/Dense>
#include <armadillo>

#include "structs.h"

// parse_dataset.cpp
std::vector<std::string> split_row(std::string sen);
void pre_process(dataset &my_dataset);
namespace gt 
{
    Eigen::MatrixXd pre_process(std::string file_path, dataset &my_dataset, const int num_col);
}
namespace imu
{
    Eigen::MatrixXd pre_process(std::string file_path, dataset &my_dataset, const int num_col);
}

// imu_noises_covariance.cpp
Eigen::Matrix<double,3,12> imu_noises_covariance(float dt);

// create_measurements.cpp
pseudoLandmarks create_measurements(const dataset& my_dataset, // pseudoLandmarks
                        const Eigen::Matrix<double,7,7>& pseudo_Q, 
                        const outSpecs outlier_specs, 
                        const int num_landmarks);
                        
arma::mat matrixxd_to_armamat(Eigen::MatrixXd eigen_A);

Eigen::VectorXd armamat_to_vectorxd(const arma::vec& arma_vec);

#endif