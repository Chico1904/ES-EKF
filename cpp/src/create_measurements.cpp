#include <Eigen/Dense>
#include <armadillo>
#include <math.h> 
#include <vector>
#include <iostream>
#include "structs.h"
#include "create_measurements.h"

/**
Takes specific ground-truth poses and adds noise. Measurements = displacement + noise, quaternions + noise
*/
pseudoLandmarks create_measurements(const dataset &my_dataset, // pseudoLandmarks
                        const Eigen::Matrix<double,7,7> pseudo_Q, 
                        const outSpecs outlier_specs, 
                        const int num_landmarks){

    Eigen::Matrix<double,-1, -1> positions {my_dataset.ground_truth(Eigen::seq(1,3), Eigen::all)}; // 
    Eigen::Matrix<double,-1, -1> orientations {my_dataset.ground_truth({7,4,5,6}, Eigen::all)}; // dataset is in qx,qy,qz,qw but we want qw first
    // positions.resize(3, my_dataset.n_timesteps);
    // orientations.resize(4, my_dataset.n_timesteps);

    // create pseudo-landmarks: based on the argument, x positions from the ground_truth will be extracted (x = num_landmarks). So in this program
    // landmarks correspond to actual positions the drone travelled to
    pseudoLandmarks landmarks {};
    Eigen::VectorXd landmarks_idx {Eigen::VectorXd::LinSpaced(num_landmarks,0,my_dataset.n_timesteps-1)};
    for(int i{0}; i<num_landmarks; ++i){
        landmarks_idx(i) = static_cast<int>(landmarks_idx(i));
    }
    landmarks.positions = positions(Eigen::all, landmarks_idx);
    landmarks.orientations = orientations(Eigen::all, landmarks_idx);

    // initialize vector of matrices: each matrix has all measurements from all landmarks @ each time instant
    Eigen::MatrixXd z_t(pseudo_Q.rows(), num_landmarks); 
    z_t.resize(pseudo_Q.rows(), num_landmarks);
    std::vector<Eigen::MatrixXd> Z;
    Eigen::Vector<double, 7> measurement;

    // sample from normal distribution to create noise. from eigen to arma object (TODO: optimize this)
    arma::vec mu = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    arma::mat pseudo_Q_arma = matrixxd_to_armamat(pseudo_Q); 


    // create and assign measurements (= true state + Gaussian noise)
    for (int i =0; i<my_dataset.n_timesteps; i++){
        Eigen::Vector3d drone_position = positions(Eigen::all, i);
        Eigen::Vector4d drone_orientation = orientations(Eigen::all, i);

        for(int j = 0; j<num_landmarks;j++){
                Eigen::Vector3d displacement = landmarks.positions(Eigen::all,j)-drone_position;
                arma::vec arma_noise = arma::mvnrnd(mu, pseudo_Q_arma);
                Eigen::VectorXd noise = armamat_to_vectorxd(arma_noise);
                Eigen::Vector3d noisy_displacement = displacement + noise(Eigen::seq(0,2));
                Eigen::Vector4d noisy_quaternion = drone_orientation + noise(Eigen::seq(3,6));
                measurement << noisy_displacement, noisy_quaternion;
                z_t(Eigen::all, j) = measurement;
        Z.push_back(z_t);
        }
    }
    landmarks.measurements = Z;

    return landmarks;
}

arma::mat matrixxd_to_armamat(Eigen::MatrixXd eigen_A) {
    arma::mat arma_B = arma::mat(eigen_A.data(), eigen_A.rows(), eigen_A.cols(),
                                 true,   
                                 false); 
    return arma_B;
}

Eigen::VectorXd armamat_to_vectorxd(const arma::vec& arma_vec) {
    Eigen::VectorXd eigen_vec(arma_vec.n_rows);
    for (size_t i = 0; i < arma_vec.n_rows; ++i) {
        eigen_vec(i) = arma_vec(i);
    }
    return eigen_vec;
}
