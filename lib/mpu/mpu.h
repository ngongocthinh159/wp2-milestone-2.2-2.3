#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define COUNT 200

// IMU parameters
extern int16_t Acc_raw_X, Acc_raw_Y, Acc_raw_Z;
extern int16_t Gyro_raw_X, Gyro_raw_Y, Gyro_raw_Z;
extern float AccX, AccY, AccZ, GyroX, GyroY, GyroZ;
extern float GyroX_calib, GyroY_calib, GyroZ_calib;
extern float AccX_calib, AccY_calib, AccZ_calib;
// long unsigned elapsedTime, time, timePrev;
extern long unsigned prev_time_mpu, dt_mpu;
extern float rad_to_deg;
extern float Acc_angle[3];
extern float Gyro_angle[3];
extern float Total_angle[3];
extern int i;

sensors_vec_t operator-(sensors_vec_t a, sensors_vec_t b);
sensors_vec_t operator-=(sensors_vec_t a, sensors_vec_t b);
sensors_event_t operator-(sensors_event_t a, sensors_event_t b);

sensors_event_t operator+(sensors_event_t a, sensors_event_t b);
sensors_vec_t operator+=(sensors_vec_t a, sensors_vec_t b);
sensors_event_t operator+(sensors_event_t a, sensors_event_t b);

sensors_vec_t operator*(sensors_vec_t a, double b);
sensors_event_t operator*(sensors_event_t a, double b);

sensors_vec_t operator/(sensors_vec_t a, double b);
sensors_event_t operator/(sensors_event_t a, double b);

void applyTime(double diff, sensors_vec_t a);
sensors_event_t mpu_calibrate_gyro(Adafruit_MPU6050 &mpu);
void setupMPU(Adafruit_MPU6050 &mpu);
void drSonMPU();
void updateMPU();
void gyro_signals(void);
void acc_signals(void);
float floatMap(float x, float in_min, float in_max, float out_min, float out_max);