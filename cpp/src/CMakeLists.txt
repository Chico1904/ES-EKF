# -------------- CMAKE TUTORIAL ----------
# First and foremost, to run something that uses libraries, you have two options:
#   -> For a quick run of one script, compile by indicating the library directory (g++ -I/path... script.cpp -o script). Then run exe with ./<FileName>
#   -> For a big run, we also need to compile and run the executable. For that, in the build directory with bash do:
#            cmake (create CMakeLists.txt) -> make    (build/compile)
#            ./<FileName>                             (run)


# Almost all CMake files should start with this
# You should always specify a range with the newest
# and oldest tested versions of CMake. This will ensure
# you pick up the best policies.
cmake_minimum_required(VERSION 3.15...4.0)

# This is your project statement. You should always list languages;
# Listing the version is nice here since it sets lots of useful variables
project(
  AE_PROJECT
  VERSION 1.0
  LANGUAGES CXX)

# add the executables to Makefile can build or smth
add_executable(${PROJECT_NAME} main.cpp imu_noises_covariance.cpp) 

# specify  directories
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ../include)


# find libraries and link to executables. If we were to specifiy a downloaded library, we would need
# to set the library path -> eg: set(CMAKE_PREFIX_PATH "/usr/include/eigen3")
find_package(Eigen3 REQUIRED)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)

include(CTest)
enable_testing()

# CMake and build advices