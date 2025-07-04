#ifndef _STRUCTS
#define _STRUCTS
#include <Eigen/Dense>
#include <vector>
#include <random>

struct user_input
{
    float dt;
    std::vector<double> pQ_variance;
    std::vector<double> Q_variance;
    std::vector<double> R_variance;
    bool outliers;
    float outliers_rate;
};

struct covM
{
    Eigen::Matrix<double,7,7> pQ;
    Eigen::Matrix<double,7,7> Q;
    Eigen::Matrix<double,3,12> R;
};

struct outSpecs
{
    bool on;
    float rate;
};

struct dataset
{
    Eigen::Matrix<double,8, -1> ground_truth;
    Eigen::Matrix<double,7, -1> imu_meas;
    int n_timesteps;
    float dt;
    double initial_timestamp;

    void resize(){
        ground_truth.resize(8, n_timesteps);
        imu_meas.resize(7, n_timesteps);
    }
};

struct pseudoLandmarks
{
    Eigen::Matrix<double,3, -1> positions;
    Eigen::Matrix<double,4, -1> orientations;
    std::vector<Eigen::MatrixXd> measurements; // Each measurement can be a dynamic matrix
};

struct normal_random_variable
{
    normal_random_variable(Eigen::MatrixXd const& covar)
        : normal_random_variable(Eigen::VectorXd::Zero(covar.rows()), covar)
    {}

    normal_random_variable(Eigen::VectorXd const& mean, Eigen::MatrixXd const& covar)
        : mean(mean)
    {
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigenSolver(covar);
        transform = eigenSolver.eigenvectors() * eigenSolver.eigenvalues().cwiseSqrt().asDiagonal();
    }

    Eigen::VectorXd mean;
    Eigen::MatrixXd transform;

    Eigen::VectorXd operator()() const
    {
        static std::mt19937 gen{ std::random_device{}() };
        static std::normal_distribution<> dist;

        return mean + transform * Eigen::VectorXd{ mean.size() }.unaryExpr([&](auto x) { return dist(gen); });
    }
};

#endif