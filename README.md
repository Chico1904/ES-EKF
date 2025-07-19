#  ES-EKF applied to UZH-FPV Drone Racing Dataset

## Overview
This repository contains the project under the EL2320 Applied Estimation course. It consists of Error State Kalman Filter applied to a drone dataset, provided by UZH [here](https://fpv.ifi.uzh.ch/). The dataset contains groundtruth and IMU data for several runs (indoor and outdoor). 

This project was initially developed in Matlab and the C++ source code is being developed as a side project. The final report, the used documentation and some results are available in this repo.

## Methodology
As the dataset only provides the ground truth and IMU data, pseudo-obstacles were introduced to supplement the dataset. In other words, Gaussian noise was added to the ground truth data and the displacement to "imaginary" landmarks was computed, yielding the measurement used in the filter. Consequently, the ES-EKF could be assessed by comparing the filter estimation to the ground truth. 

## Some results
The *results* folder contains the results presented in the final report. Some of those plots are presented below:
- Real trajectory *vs* trajectory estimated by the filter
![trajectory](https://github.com/user-attachments/assets/c580d1a2-a25d-446f-82f4-ab67f88a5c49)

- Position error over time
![position_error](https://github.com/user-attachments/assets/b7eb0375-fc49-4fb0-8532-f5e81983e588)



