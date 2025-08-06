#pragma once

#include <Eigen/Dense>
#include <armadillo>

template <typename V, typename M> 
V sample_randn(const V& mu, M& cov);

template <typename T>
Eigen::Matrix3d skew(const T t);

template <typename V, typename M> 
V sample_randn(const V& mu, M& cov){
    // Note: M cannot be constant (check compiler error in arma functions)
    const arma::mat arma_cov {arma::mat(cov.data(), cov.rows(), cov.cols(),
                                 true,   
                                 false)};
    const arma::vec arma_mu {arma::vec(mu.data(), mu.size())};
    const arma::vec arma_sample {arma::mvnrnd(arma_mu, arma_cov)};
    V eigen_vec(arma_sample.n_rows);
    for (size_t i = 0; i < arma_sample.n_rows; ++i) {
        eigen_vec(i) = arma_sample(i);
    }
    return eigen_vec;
}

template <typename T>
Eigen::Matrix3d skew(const T t){
    Eigen::Matrix3d t_hat;
    t_hat << 0, -t(2), t(1),
        t(2), 0, -t(0),
        -t(1), t(0), 0;
    return t_hat;

}
