#include "structs.h"
#include "pre_process.h"
#include "ekf.h"

EKF::EKF(user_input my_input){
    const float dt = my_input.dt;
    data.dt = dt;

    outliers = my_input.outliers;
    outliers_rate = my_input.outliers_rate;

    cov.pQ.setZero();
    Eigen::Vector<double, 7> eigen_pQ_variance(my_input.pQ_variance.data());
    cov.pQ.diagonal() = eigen_pQ_variance;

    cov.Q.setZero();
    Eigen::Vector<double, 7> eigen_Q_variance(my_input.Q_variance.data());
    cov.Q.diagonal() = eigen_Q_variance;

    cov.R.setZero();
    Eigen::Vector<double, 12> eigen_R_variance(my_input.R_variance.data());
    for (int i = 0; i < 12; i += 3){
        cov.R(Eigen::seq(0,2), Eigen::seq(i,i+2)).diagonal() = eigen_R_variance(Eigen::seq(i,i+2));
    }
    
}
   
 //  ----------------------- Methods
void EKF::pre_filter_loop(){

    // Create dataset: groud_truth (pos+ori), IMU measurements, dt and num_datapoints
    pre_process(data); // Parse data based on timestep -> get ground_truth and imu

    // get measurement and model covariance matrices considered by the Kalman Filter
    Eigen::Matrix<double,3,12> R = imu_noises_covariance(dt);

    // Generate pseudo measurements -> add random noise
    outSpecs out;
    pseudolandmarks = create_measurements(data, cov.pQ, out, 5);
    std::cout << pseudolandmarks.measurements.back();
    };

void EKF::initialize_state(){
    preds.sigma.setIdentity();

    // initial nominal state (belief)
    preds.muNom(Eigen::seq(0,2)) = data.ground_truth(Eigen::seq(1,3), 0);
    preds.muNom(Eigen::seq(3,6)) = data.ground_truth({7,4,5,6}, 0);
    preds.muNom(Eigen::seq(16,18)) << 0,0,-9.81;

    // set quaternion and corresponding rotation matrix (dataset is in qx,qy,qz,qw)
    preds.muQuat.x() = data.ground_truth(4,0);
    preds.muQuat.y() = data.ground_truth(5,0);
    preds.muQuat.z() = data.ground_truth(6,0);
    preds.muQuat.w() = data.ground_truth(7,0);
    Eigen::Matrix3d R = preds.muQuat.normalized().toRotationMatrix();

    // set initial velocity and imu acceleration bias
    preds.muNom(Eigen::seq(3,5)) = (data.ground_truth(Eigen::seq(1,3), 1) - data.ground_truth(Eigen::seq(1,3), 0))/data.dt;
    Eigen::Vector3d v_p = (data.ground_truth(Eigen::seq(1,3), 2) - data.ground_truth(Eigen::seq(1,3), 1))/data.dt;
    preds.muNom(Eigen::seq(10,12)) = data.imu_meas(Eigen::seq(4,6),0) - R.transpose()*((v_p-data.ground_truth(Eigen::seq(1,3), 0))/data.dt - data.ground_truth(Eigen::seq(1,3), 0));

    // set imu angular velocity bias





}
