#include <Eigen/Dense>
#include <math.h> 
#include <iostream>
/* 
see https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model
 MODE
   1: simulated ground truth with additional measurement and process noise
    measurement noise comes from createMeasurements, process noise is added
    in the function NoisesCovariance.
   2: true ground truth 
*/

Eigen::Matrix<double,3,12> imu_noises_covariance(float dt){
    
    float f {1000}; // rate, Hz
    // linear acceleration and angular velocities noise
    float sigma_a {0.1};  // accelerometer_noise_density
    float sigma_g  {0.05}; // gyroscope_noise_density

    // linear acceleration and angular velocities bias noise
    float sigma_ba {0.002}; //accelerometer_random_walk
    float sigma_bg {4.0e-05}; //gyroscope_random_walk

    Eigen::Matrix<double,3,3> I {};
    I << 1,0,0,0,1,0,0,0,1;

    Eigen::Matrix<double,3,12> imu_cov(I.rows(), 4*I.cols());
    imu_cov << pow(sigma_a,2)*I, pow(sigma_g,2)*I, pow(sigma_ba,2)*I, pow(sigma_bg,2)*I;
    //std::cout << "imu cov =" << imu_cov;
    return imu_cov;
}



