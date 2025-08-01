#include <cmath>
#include "structs.h"
/*
Use the motion model to predic mu (nominal & error) and sigma. It uses the previous
state vector (nominal & error), covariance matrix and the imu measurements (in data)
*/
void predict(state& state, const dataset& data){

    // Compute acceleration and rotation matrix
    const Eigen::Matrix3d R = state.muQuat.toRotationMatrix();
    const Eigen::Vector3d accel = R*(data.imu_meas(Eigen::seq(3,5))-state.muNom(Eigen::seq(10,12))) + state.muNom(Eigen::seq(16,18)); // assuming gravity vector is in the world frame
    
    // Update nominal state (pos, vel, quat, etc)
    state.muNom(Eigen::seq(0,2)) = state.muNom(Eigen::seq(0,2)) + data.dt*state.muNom(Eigen::seq(3,5)) + 0.5*accel*std::pow(data.dt,2);
    state.muNom(Eigen::seq(3,5)) = state.muNom(Eigen::seq(3,5)) + data.dt*accel;

    const Eigen::Vector3d angle = data.dt*(data.imu_meas(Eigen::seq(0,2))-state.muNom(Eigen::seq (13,15)));
    Eigen::Quaterniond q_omega {};
    q_omega.x() = std::sin(angle.norm())*angle(0)/angle.norm();
    q_omega.y() = std::sin(angle.norm())*angle(1)/angle.norm();
    q_omega.z() = std::sin(angle.norm())*angle(2)/angle.norm();
    q_omega.w() = std::cos(angle.norm()/2);
    state.muQuat = state.muQuat*q_omega;
    state.muQuat = state.muQuat.normalized();
    
    // Update error state

    // Update covariance
}