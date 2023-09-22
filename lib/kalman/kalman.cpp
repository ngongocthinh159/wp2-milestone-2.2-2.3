#include <kalman.h>
#include <utils.h>
#include <Arduino.h>

// initial error rate (degree/s)
// const double initialEstimateErr = 0.1;
const double initialEstimateErr = 1;
// double KalmanEstimateErr = initialEstimateErr;

// estimated error rate per second (degree/s)
// check matlab deg_gyro_x (or y or z), see how much the error is for 1 sec
// double InputPredict = 0.01; 
double InputPredict = 0.001; 

// measurement noise from accelerometer (degree)
// double measurementNoise = 0.001;
double measurementNoise = 0.001;

/*!
    @brief 1D Kalman filter
    @param KalmanEstimate: output of Kalman filter (degree)
    @param KalmanEstimateErr: error of Kalman filter (degree)
    @param InputRate: rate of change of gyro (degree/s)
    @param KalmanMeasurement: angle measurement from accelerometer (degree)
    @param timeDifference: time difference between measurements (ms)
    @return KalmanGain: Kalman gain for debugging
*/
double kalman_1d(double &KalmanEstimate, double &KalmanEstimateErr, double InputRate, double KalmanMeasurement, double timeDifference) {
    double KalmanPredict = KalmanEstimate + timeDifference * InputRate;

    double KalmanError = KalmanEstimateErr + timeDifference * InputPredict;

    double KalmanGain = KalmanError / (KalmanError + measurementNoise);

    KalmanEstimate = KalmanPredict + KalmanGain * (KalmanMeasurement - KalmanPredict);
 
    KalmanEstimateErr = (1 - KalmanGain) * KalmanEstimateErr;

    return KalmanGain;
}