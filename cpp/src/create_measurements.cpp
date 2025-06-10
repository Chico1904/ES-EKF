#include <Eigen/Dense>
#include <math.h> 
#include <vector>
#include <iostream>
#include "structs.h"


pseudoLandmarks create_measurements(const Eigen::Matrix<double,7, -1> ground_truth, 
                        const Eigen::Matrix<double,7,7> pseudo_Q, 
                        const outSpecs outlier_specs, 
                        const int num_landmarks){

    Eigen::Matrix<double,3, -1> positions {ground_truth(Eigen::seq(1,3), Eigen::all)}; // 
    Eigen::Matrix<double,4, -1> orientations {ground_truth({7,4,5,6}, Eigen::all)}; // dataset is in qx,qy,qz,qw but we want qw first

    // create pseudo-landmarks: based on the argument, num_landmarks positions from the ground_truth will be extracted. So in this program
    // landmarks correspond to actual positions the drone travelled to
    const int n_timesteps {positions.cols()}; //in case we change dt
    pseudoLandmarks landmarks {};
    Eigen::VectorXd landmarks_idx {Eigen::VectorXd::LinSpaced(num_landmarks,0,n_timesteps-1)};
    std::cout << "Landmarks indexes = " << landmarks_idx << '\n';
    landmarks.positions = positions(Eigen::all, landmarks_idx);

    // initialize vector of matrices: each matrix has all measurements from all landmarks @ each time instant
    Eigen::MatrixXd z_t(pseudo_Q.rows(), num_landmarks); 
    std::vector<Eigen::MatrixXd> Z;

    // create and assign measurements (= true state + Gaussian noise)
    Eigen::VectorXi ind_pos(3); ind_pos<<0,1,2;
    Eigen::VectorXi ind_ori(4); ind_ori<<3,4,5,6;
    normal_random_variable noise_struct  { pseudo_Q };

    for (int i =0; i<n_timesteps; i++){
        Eigen::Vector3d drone_position {positions(Eigen::all, i)};
        Eigen::Vector4d drone_orientation {orientations(Eigen::all, i)};

        for(int j = 0; j<num_landmarks;j++){
                Eigen::Vector3d displacement {landmarks.positions(Eigen::all,j)-drone_position};
                Eigen::Vector3d noise {noise_struct()};
                Eigen::Vector3d noisy_displacement {displacement + noise(ind_pos)};
                Eigen::Vector3d noisy_quaternion {drone_orientation + noise(ind_ori)};
                z_t(Eigen::all, j) = noisy_displacement, noisy_quaternion;
        Z[i] = z_t;

        }
    }
    landmarks.measurements = Z;

    return landmarks;
   
}


    // how to use gaussian struct 
//     int size = 2;
//     Eigen::MatrixXd covar(size,size);
//     covar << 1, .5,
//             .5, 1;

//     normal_random_variable sample { covar };

//     std::cout << sample() << std::endl;
//     std::cout << sample() << std::endl;