#include <cmath>
#include "structs.h"
#include "utils.h"
/*
Use the motion model to predic mu (nominal & error) and sigma. It uses the previous
state vector (nominal & error), covariance matrix and the imu measurements (in data)

TODO: remove quaternion from muErr
*/
void predict(state& state, const dataset& data, Eigen::Matrix<double,3,12>& R){
    using Eigen::seq;

    // Compute acceleration and rotation matrix
    const Eigen::Matrix3d R = state.muQuat.toRotationMatrix();
    const Eigen::Vector3d accel = R*(data.imu_meas(seq(4,6))-state.muNom(seq(10,12))) + state.muNom(seq(16,18)); // assuming gravity vector is in the world frame
    
    // Update nominal state (pos, vel, quat, etc)
    state.muNom(seq(0,2)) = state.muNom(seq(0,2)) + data.dt*state.muNom(seq(3,5)) + 0.5*accel*std::pow(data.dt,2);
    state.muNom(seq(3,5)) = state.muNom(seq(3,5)) + data.dt*accel;

    const Eigen::Vector3d angle = data.dt*(data.imu_meas(seq(1,3))-state.muNom(seq (13,15)));
    Eigen::Quaterniond q_omega {};
    q_omega.x() = std::sin(angle.norm())*angle(0)/angle.norm();
    q_omega.y() = std::sin(angle.norm())*angle(1)/angle.norm();
    q_omega.z() = std::sin(angle.norm())*angle(2)/angle.norm();
    q_omega.w() = std::cos(angle.norm()/2);
    state.muQuat = state.muQuat*q_omega;
    state.muQuat = state.muQuat.normalized();

    // Update error state (pos, vel, quat, etc)
    const Eigen::Vector3d mu = {}; //null vector to sample noise from

    state.muErr(seq(0,2)) = state.muErr(seq(0,2)) + data.dt*state.muErr(seq(3,5)); // pos

    Eigen::Matrix3d imu_cov {R(Eigen::all, seq(0,2))}; // vel
    state.muErr(seq(3,5)) = state.muErr(seq(3,5)) + 
    (-R*skew(data.imu_meas(seq(4, 6))-state.muNom(seq(10,12)))*state.muErr(seq(6,8)) - R*state.muErr(seq(9,11)) + state.muErr(seq(15,17)))*data.dt 
    + sample_randn(mu, imu_cov);

    imu_cov = R(Eigen::all, seq(3,5)); // orientation
    Eigen::Vector3d omega {data.imu_meas(seq(1,3)) - state.muNom(seq(13,15))};
    Eigen::Matrix3d RR {};
    RR.setIdentity();
    if (omega.norm()*data.dt > 1e-6){
        RR += std::sin(omega.norm()*data.dt)*skew(omega.normalized()) //Rodrigues' formula
        + (1-std::sin(omega.norm()*data.dt))*skew(omega.normalized())* skew(omega.normalized());
    }
    state.muErr(seq(6,8)) = RR.transpose()*state.muErr(seq(6,8)) - state.muErr(seq(12,14))*data.dt + sample_randn(mu, imu_cov);
    
    imu_cov = R(Eigen::all, seq(6,8));
    state.muErr(seq(9,11)) += sample_randn(mu, imu_cov); // lin accel

    imu_cov = R(Eigen::all, seq(9,11));
    state.muErr(seq(12,14)) += sample_randn(mu, imu_cov); // ang vel bias

    // Update covariance
};