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
    dataset my_dataset {};        
    pseudoLandmarks my_pseudolandmarks {};
    covM my_cov {};

    //  ----------------------- Constructor
    EKF(user_input my_input);

    // Methods
    void pre_filter_loop();

};

#endif