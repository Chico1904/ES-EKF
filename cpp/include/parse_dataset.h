#ifndef _PARSEDATA
#define _PARSEDATA
#include <iostream>
#include <math.h> 
#include <vector>
#include <fstream>
#include <filesystem>
#include <Eigen/Dense>
#include <ctime>
#include "structs.h"

// Functions
std::vector<std::string> split_row(std::string sen);

void pre_process(dataset &my_dataset);

Eigen::MatrixXd pre_process_gt(std::string file_path, dataset &my_dataset, const int num_col);

Eigen::MatrixXd pre_process_imu(std::string file_path, dataset &my_dataset, const int num_col);

#endif