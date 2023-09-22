extern const double initialEstimateErr;
extern double KalmanEstimateErr;
extern double InputPredict;
extern double MeasurementNoise;

double kalman_1d(double &KalmanEstimate, double &KalmanEstimateErr, double InputRate, double KalmanMeasurement, double timeDifference);