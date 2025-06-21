#include "imu_noises_covariance.h"
#include "parse_dataset.h"
#include "create_measurements.h"
#include "structs.h"

int main(){

    // Define program variables (add GUI later on)
    const float dt {2.0};

    Eigen::Matrix<double,7,7> pseudo_Q; // model covariance matrix (model, ie, IMU)
    float cov_pQ {0.1};
    pseudo_Q.setIdentity();
    pseudo_Q *= cov_pQ;

    Eigen::Matrix<double,7,7> pseudo_R; // noise covariance matrix (measurements)
    float cov_pR {0.1};
    pseudo_R.setIdentity();
    pseudo_R *= cov_pQ;

    // Create dataset: groud_truth (pos+ori), IMU measurements, dt and num_datapoints
    dataset my_dataset {};
    my_dataset.dt = dt;
    pre_process(my_dataset); // Parse data based on timestep -> get ground_truth and imu
 
    // get measurement and model covariance matrices considered by the Kalman Filter
    Eigen::Matrix<double,3,12> R = imu_noises_covariance(dt);

    float cov {0.1};
    Eigen::Matrix<double,7,7> Q;
    Q.setIdentity();
    Q *= cov;

    // Generate pseudo measurements -> add random noise
    outSpecs out;
    pseudoLandmarks res = create_measurements(my_dataset, pseudo_Q, out, 5);
    return 0;

}