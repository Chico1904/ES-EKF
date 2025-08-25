#include <cmath>
#include "structs.h"
#include "utils.h"
/*
Use the motion model to predic mu (nominal & error) and sigma. It uses the previous
state vector (nominal & error), covariance matrix and the imu measurements (in data)

TODO: remove quaternion from muErr
*/
void predict(const float dt, state& state, const Eigen::Vector<double, 7>& imu, Eigen::Matrix<double,3,12>& cov_R){
    using Eigen::seq;

    // Compute acceleration and rotation matrix
    const Eigen::Matrix3d R {state.muQuat.toRotationMatrix()};
    const Eigen::Vector3d accel {R*(imu(seq(4,6))-state.muNom(seq(10,12))) + state.muNom(seq(16,18))}; // assuming gravity vector is in the world frame
    
    // 1- Update nominal state (pos, vel, quat, etc)
    state.muNom(seq(0,2)) = state.muNom(seq(0,2)) + dt*state.muNom(seq(3,5)) + 0.5*accel*std::pow(dt,2);
    state.muNom(seq(3,5)) = state.muNom(seq(3,5)) + dt*accel;

    const Eigen::Vector3d angle = dt*(imu(seq(1,3))-state.muNom(seq(13,15)));
    Eigen::AngleAxisd aa(angle.norm(), angle.normalized());
    Eigen::Quaterniond q_omega(aa); // if this causes errors, see below
    // q_omega.x() = std::sin(0.5*angle.norm())*angle(0)/angle.norm();
    // q_omega.y() = std::sin(0.5*angle.norm())*angle(1)/angle.norm();
    // q_omega.z() = std::sin(0.5*angle.norm())*angle(2)/angle.norm();
    // q_omega.w() = std::cos(0.5*angle.norm());
    state.muQuat = state.muQuat*q_omega;
    state.muQuat = state.muQuat.normalized();

    // 2- Update error state (pos, vel, quat, etc)
    const Eigen::Vector3d mu = {}; //null vector to sample noise from

    state.muErr(seq(0,2)) = state.muErr(seq(0,2)) + dt*state.muErr(seq(3,5)); // pos

    Eigen::Matrix3d imu_cov {cov_R(Eigen::all, seq(0,2))}; // vel
    state.muErr(seq(3,5)) = state.muErr(seq(3,5)) + 
    (-R*skew(imu(seq(4, 6))-state.muNom(seq(10,12)))*state.muErr(seq(6,8)) - R*state.muErr(seq(9,11)) + state.muErr(seq(15,17)))*dt 
    + sample_randn(mu, imu_cov);

    imu_cov = R(Eigen::all, seq(3,5)); // orientation
    Eigen::Vector3d omega {imu(seq(1,3)) - state.muNom(seq(13,15))};
    Eigen::Matrix3d RR {};
    RR.setIdentity();
    if (omega.norm()*dt > 1e-6){
        RR += std::sin(omega.norm()*dt)*skew(omega.normalized()) //Rodrigues' formula
        + (1-std::sin(omega.norm()*dt))*skew(omega.normalized())* skew(omega.normalized());
    }
    state.muErr(seq(6,8)) = RR.transpose()*state.muErr(seq(6,8)) - state.muErr(seq(12,14))*dt + sample_randn(mu, imu_cov);
    
    imu_cov = R(Eigen::all, seq(6,8));
    state.muErr(seq(9,11)) += sample_randn(mu, imu_cov); // lin accel

    imu_cov = R(Eigen::all, seq(9,11));
    state.muErr(seq(12,14)) += sample_randn(mu, imu_cov); // ang vel bias

    // 3- Update covariance
    Eigen::Matrix <double,18,18> Fx ;
    Eigen::Matrix<double,18,12> Fi;
    Eigen::Matrix<double,12,12> block_R;

    Eigen::Matrix3d a_sk {skew(imu(seq(4,6)))};
    Eigen::Matrix3d I {Eigen::Matrix3d::Identity(3,3)};
    Eigen::Matrix3d Z {Eigen::Matrix3d::Zero(3,3)};
    Eigen::Matrix<double,3,9> Z39 = Eigen::Matrix<double,3,9>::Zero();
    Eigen::Matrix<double,3,6> Z36 = Eigen::Matrix<double,3,6>::Zero();

    Fx <<
        I,       I*dt,  Z,              Z,       Z,       Z,
        Z,       I,     -R*a_sk*dt,     -R*dt,   Z,       I*dt,
        Z,       Z,     RR.transpose(), Z,       -I*dt,   Z,
        Z,       Z,     Z,              I,       Z,       Z,
        Z,       Z,     Z,              Z,       I,       Z,
        Z,       Z,     Z,              Z,       Z,       I;

    Fi <<
        Eigen::Matrix<double,3,12>::Zero(),   
        I, Z39,                               
        Z, I, Z36,                            
        Z36, I, Z, 
        Z39, I, 
        Eigen::Matrix<double,3,12>::Zero();    

    block_R <<
        cov_R.block<3,3>(0,0),Eigen::Matrix<double,3,9>::Zero(),
        Z, cov_R.block<3,3>(0,3), Z36,
        Z36, cov_R.block<3,3>(0,6), Z,
        Z39, cov_R.block<3,3>(0,9);

    state.sigma = Fx*state.sigma*Fx.transpose() + Fi*block_R*Fi.transpose();
};