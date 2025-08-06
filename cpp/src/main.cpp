#include "pre_process.h"
#include "structs.h"
#include "ekf.h"

int main(){
    
    user_input my_input{};
    my_input.dt = 0.2;
    my_input.outliers = true;
    my_input.outliers_rate = 0.2;
    my_input.pQ_variance = std::vector<double>(7, 0.1);
    my_input.Q_variance = std::vector<double>(7, 0.1);
    my_input.R_variance = std::vector<double>(12, 0.1);
    EKF ekf(my_input);

    // Get ground_truth, IMU and create pseudo measurements
    ekf.pre_filter_loop();

    // Initialize nominal and error states
    ekf.initialize_state();


    return 0;

};
