#include "structs.h"
#include "pre_process.h"
#include "ekf.h"

EKF::EKF(user_input my_input){
    const float dt = my_input.dt;
    my_dataset.dt = dt;

    outliers = my_input.outliers;
    outliers_rate = my_input.outliers_rate;

    my_cov.pQ.setZero();
    Eigen::Vector<double, 7> eigen_pQ_variance(my_input.pQ_variance.data());
    my_cov.pQ.diagonal() = eigen_pQ_variance;

    my_cov.Q.setZero();
    Eigen::Vector<double, 7> eigen_Q_variance(my_input.Q_variance.data());
    my_cov.Q.diagonal() = eigen_Q_variance;

    my_cov.R.setZero();
    Eigen::Vector<double, 12> eigen_R_variance(my_input.R_variance.data());
    for (int i = 0; i < 12; i += 3){
        my_cov.R(Eigen::seq(0,2), Eigen::seq(i,i+2)).diagonal() = eigen_R_variance(Eigen::seq(i,i+2));
    }
    
}
   
 //  ----------------------- Methods
void EKF::pre_filter_loop(){

    // Create dataset: groud_truth (pos+ori), IMU measurements, dt and num_datapoints
    pre_process(my_dataset); // Parse data based on timestep -> get ground_truth and imu

    // get measurement and model covariance matrices considered by the Kalman Filter
    Eigen::Matrix<double,3,12> R = imu_noises_covariance(dt);

    // Generate pseudo measurements -> add random noise
    outSpecs out;
    pseudoLandmarks res = create_measurements(my_dataset, my_cov.pQ, out, 5);

    };
