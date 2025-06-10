#include "parse_dataset.h"
#include "structs.h"
void pre_process(dataset &my_dataset){
    clock_t start = clock();

    // parse ground truth
    std::string current_path = std::filesystem::current_path();
    std::string dataset_path = current_path + "/datasets/Dataset9_indoor/davis_groundtruth.txt";
    Eigen::MatrixXd ground_truth = pre_process_gt(dataset_path, my_dataset, 8);

    // parse IMU
    dataset_path = current_path + "/datasets/Dataset9_indoor/davis_imu.txt";
    Eigen::MatrixXd imu_data = pre_process_imu(dataset_path, my_dataset, 7);

    // Assign to dataset structure
    my_dataset.ground_truth = ground_truth;
    my_dataset.imu_meas = imu_data;

    clock_t end = clock();
    double elapsed = double(end - start) / CLOCKS_PER_SEC;
    std::cout << "Parsing time: " << elapsed << " seconds." << std::endl;

}

Eigen::MatrixXd pre_process_gt(std::string dataset_path, dataset &my_dataset, const int num_col){
    
    // Get number of datapoints //TODO: improve this
    Eigen::MatrixXd matrix2parse {};
    std::ifstream count_dataset(dataset_path);
    double init_timestamp {};
    int i {-1};
    for (std::string line; std::getline(count_dataset, line);){
        std::vector<std::string> row = split_row(line);
        if (i == -1){
            ++ i;
        }
        else if (i == 0){
            init_timestamp = std::stod(row[0]);
            my_dataset.initial_timestamp = init_timestamp;
            ++ i;
            matrix2parse.resize(num_col, i); 
        }
        else if ((std::round((std::stod(row[0]) - init_timestamp)* 1000.0) / 1000.0)/my_dataset.dt >= i){
            ++ i;
            matrix2parse.resize(num_col, i); 
        }
    }

    // Define number of timesteps based on ground truth
    const int n_timesteps {i};  
    my_dataset.n_timesteps = n_timesteps;   
    matrix2parse.resize(num_col, n_timesteps);        

    // Assign datapoint to row in matrix
    std::ifstream parse_dataset(dataset_path);
    i = -1;
    int j {0};
    for (std::string line; std::getline(parse_dataset, line);){
        std::vector<std::string> row = split_row(line);
        if (i == -1){
            ++ i;
        }
        else if (i == 0){
            init_timestamp = std::stod(row[0]);
            j = 0;
            for (std::string w : row) {
                
                if ( j == 0){
                    matrix2parse(j,i) = std::stod(w) - init_timestamp;
                }
                else{
                    matrix2parse(j,i) = std::stod(w);
                }
                ++j;
            }
            ++ i;
        }
        else if ((std::round((std::stod(row[0]) - init_timestamp)* 1000.0) / 1000.0)/my_dataset.dt >= i)
        {
           j = 0;
            for (std::string w : row) {
                if (j == 0){
                    matrix2parse(j,i) = std::round((std::stod(w) - init_timestamp)* 1000.0) / 1000.0; 
                }
                else{
                    matrix2parse(j,i) = std::stod(w);
                }
                ++j;
            }
            ++ i;
        }
        
    }
    parse_dataset.close();

    return matrix2parse;
}

Eigen::MatrixXd pre_process_imu(std::string dataset_path, dataset &my_dataset, const int num_col){

    Eigen::MatrixXd matrix2parse {};
    matrix2parse.resize(num_col, my_dataset.n_timesteps);
    std::ifstream parse_dataset(dataset_path);
    int i {-1};
    int j {0};
    for (std::string line; std::getline(parse_dataset, line);){
        std::vector<std::string> row = split_row(line);
        if (i >= my_dataset.n_timesteps-1){
            break;
        }

        else if (i == -1){
            ++ i;
        }

        else if ((std::stod(row[1]) - my_dataset.initial_timestamp)/ my_dataset.dt >= i){
            j = 0;
            for (std::string w : row) {
                if (j == 1){
                    matrix2parse(j-1,i) = std::round((std::stod(w) - my_dataset.initial_timestamp)* 1000.0) / 1000.0; 
                }
                else if (j > 1){
                    matrix2parse(j-1,i) = std::stod(w);
                }
                ++j;         
            }
            ++ i;
        }

        
    }
    parse_dataset.close();
    return matrix2parse;
}

std::vector<std::string> split_row(std::string sen) {
  
    // Create a stringstream object
    std::stringstream ss(sen);
    
    // Variable to hold each word
    std::string word;
    
    // Vector to store the words
    std::vector<std::string> words;
    
    // Extract words using getline with space as the delimiter
    while (getline(ss, word, ' ')) {
      
        // Add the word to the vector
        words.push_back(word);
    }
    
    return words;
}

