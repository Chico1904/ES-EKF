#ifndef _IMUNOISECOV
#define _IMUNOISECOV
#include <Eigen/Dense>

Eigen::Matrix<double,3,12> imu_noises_covariance(float dt);

#endif