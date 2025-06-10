#include "imu_noises_covariance.h"
#include "parse_dataset.h"
#include "structs.h"



int main(){

    // Define program variables (add GUI later on)
    const float dt {0.01};
    int pseudo_Q = 0;// model covariance matrix (model, ie, IMU) 
    int pseudo_R = 0; // noise covariance matrix (measurements)

    // Create dataset: groud_truth (pos+ori), IMU measurements, dt and num_datapoints
    dataset my_dataset {};
    my_dataset.dt = dt;
    pre_process(my_dataset); // Parse data based on timestep -> get ground_truth and imu
 
    
    // Generate pseudo measurements and covariance matrix (R, model)
    return 0;

}