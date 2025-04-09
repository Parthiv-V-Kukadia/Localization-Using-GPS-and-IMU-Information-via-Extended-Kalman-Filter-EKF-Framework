# Localization Using GPS and IMU Information via Extended Kalman Filter (EKF) Framework
This project implements an Extended Kalman Filter (EKF) to estimate the North-East-Down (NED) position and velocity of an Unmanned Aerial Vehicle (UAV) using noisy sensor data. The filter fuses data from an Inertial Measurement Unit (IMU) (specifically accelerometer readings) and a Global Positioning System (GPS) receiver.

## Overview

The MATLAB script performs the following steps:

1.  **Data Loading:** Loads sensor data from a `UAV.mat` file. This file contains time-series data for:
    * Accelerometer readings (acx, acy, acz)
    * Euler angles (phi, tht, psi) computed by the drone's onboard computer (Roll, Pitch, Yaw)
    * GPS information (fix, eph, epv, lat, lon, alt, gps\_nSat)
    * Motor signals (out1, out2, out3, out4)
    * Earth ellipsoid parameters (a, b, e2)
    * Initial UAV position as a GPS reference (lat\_0, lon\_0, alt\_0)

2.  **Data Visualization (Initial Plots):** Generates initial plots of the raw sensor data, including:
    * Accelerometer readings along the x, y, and z axes.
    * Euler angles (Roll, Pitch, Yaw) over time.
    * GPS data (Longitude, Latitude, Altitude, Number of Satellites, Horizontal Variance, Vertical Variance).
    * Motor control signals for the four motors.

3.  **GPS to NED Conversion:** Converts the raw GPS Latitude, Longitude, and Altitude measurements into local North-East-Down (NED) coordinates. This involves:
    * Converting Latitude and Longitude from degrees to radians.
    * Defining a reference GPS position (the initial UAV position).
    * Implementing functions to convert Geodetic coordinates (Latitude, Longitude, Altitude) to Earth-Centered, Earth-Fixed (ECEF) coordinates and then from ECEF to NED coordinates.

4.  **Extended Kalman Filter Implementation:** Implements an EKF to estimate the UAV's state during a specific flight segment (second take-off and landing). The state vector includes:
    * NED Position (x\_n, y\_n, z\_n)
    * NED Velocity (v\_x, v\_y, v\_z)
    * Accelerometer Bias (b\_x, b\_y, b\_z)

    The EKF process involves:
    * **Initialization:** Defining the initial state, initial covariance matrix (P0), and measurement noise covariance matrix (R) based on GPS horizontal and vertical variance.
    * **Time Interval Definition:** Identifying the time interval of interest based on the motor signals (indicating take-off and landing times) and defining the corresponding data indices.
    * **System Matrices:** Defining the State Transition Matrix (F) and the Control Input Matrix (G), which incorporate the rotation from the NED frame to the ground frame (using Euler angles) and the time step (dt).
    * **Process Noise Covariance Matrix (Q):** Defining the process noise based on assumed noise characteristics of the IMU.
    * **Prediction Step:** Predicting the next state (x\_pred) and the predicted covariance matrix (P\_pred) using the system dynamics and the previous state estimate.
    * **Measurement Update Step (Correction):**
        * Defining the Measurement Model (y) based on the GPS-derived NED positions.
        * Defining the Output Matrix (H) to relate the measurement to the state vector (only position is directly measured by GPS).
        * Calculating the Residual Covariance (S) and the Optimal Kalman Gain (W).
        * Updating the state estimate (x0) and the covariance matrix (P0) using the Kalman gain and the measurement residual.
        * The script implements a lower update rate for GPS measurements (every other time step) to reflect the typical behavior of GPS updates being slower than IMU updates. During IMU-only updates, only the prediction step is performed.
    * **Storing Estimates:** Storing the estimated state at each time step.

5.  **Result Visualization (EKF Plots):** Generates plots to evaluate the performance of the EKF:
    * Comparison of raw GPS Longitude, Latitude, and Altitude before and after the defined processing window.
    * Plot of the NED positions derived from the raw GPS measurements over time.
    * Comparison of the EKF-estimated NED positions with the NED positions derived directly from GPS.
    * Comparison of the EKF-estimated NED velocities with the velocities calculated from the differenced GPS-derived NED positions.
    * Plot of the NED accelerations calculated from the differenced GPS-derived NED positions.
    * Plot of the estimated accelerometer biases (bx, by, bz) over time.

## Files Included

* `UAV.mat`: MATLAB data file containing the UAV sensor data.
* MATLAB script (the provided code).

## How to Run the Script

1.  **Save the code:** Save the provided MATLAB code as a `.m` file (e.g., `uav_ekf.m`).
2.  **Ensure `UAV.mat` is present:** Make sure the `UAV.mat` file is in the same directory as the MATLAB script.
3.  **Open MATLAB:** Launch the MATLAB environment.
4.  **Navigate to the directory:** In the MATLAB command window, navigate to the directory where you saved the `.m` file and `UAV.mat`.
5.  **Run the script:** Execute the script by typing `uav_ekf` in the command window and pressing Enter.

MATLAB will then:

* Load the data from `UAV.mat`.
* Generate the initial plots of the raw sensor data.
* Convert the GPS data to NED coordinates.
* Implement the Extended Kalman Filter to estimate the UAV's state.
* Generate the result plots comparing the estimated and measured NED positions and velocities, as well as the estimated accelerometer biases.

By examining the generated plots, you can analyze the performance of the EKF in estimating the UAV's trajectory and sensor biases. The comparison between the EKF estimates and the GPS-derived values will indicate the effectiveness of the filter in reducing noise and providing a smoother, potentially more accurate estimate of the UAV's state.
