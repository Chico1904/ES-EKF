#ifndef _EKF
#define _EKF
#include <math.h> 
#include <vector>
#include <Eigen/Dense>
#include "structs.h"

class EKF {       
  public:     
    //  ----------------------- Attributes      
    float dt {};
    bool outliers {};
    float outliers_rate {};
    
    covMatrices cov{};
    dataset data {};        
    pseudoLandmarks pseudolandmarks {};

    predictions preds{};

    //  ----------------------- Constructor
    EKF(user_input my_input);

    // Methods
    void pre_filter_loop();

    void initialize_state();

};

#endif