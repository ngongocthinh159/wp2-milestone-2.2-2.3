#include <mpu.h>

// IMU parameters
int16_t Acc_raw_X, Acc_raw_Y, Acc_raw_Z;
int16_t Gyro_raw_X, Gyro_raw_Y, Gyro_raw_Z;
float AccX, AccY, AccZ, GyroX, GyroY, GyroZ;
float GyroX_calib, GyroY_calib, GyroZ_calib;
float AccX_calib, AccY_calib, AccZ_calib;
// long unsigned elapsedTime, time, timePrev;
long unsigned prev_time_mpu, dt_mpu;
float rad_to_deg = 180 / 3.141592654;
float Acc_angle[3];
float Gyro_angle[3] = {0, 0, 0};
float Total_angle[3] = {0, 0, 0};
int i;

// Serial configuration parameters
unsigned long curr_time = 0, prev_time_com = 0, dt_com = 50000;  // time interval in us

sensors_vec_t operator-(sensors_vec_t a, sensors_vec_t b) {
    sensors_vec_t c = a;
    c.x -= b.x;
    c.y -= b.y;
    c.z -= b.z;

    return c;
}

sensors_vec_t operator-=(sensors_vec_t a, sensors_vec_t b) {
    return a - b;
}

sensors_event_t operator-(sensors_event_t a, sensors_event_t b) {
    sensors_event_t c = a;
    c.acceleration = a.acceleration - b.acceleration;
    c.gyro = a.gyro - b.gyro;
    return c;
}

sensors_vec_t operator+(sensors_vec_t a, sensors_vec_t b) {
    sensors_vec_t c = a;
    c.x += b.x;
    c.y += b.y;
    c.z += b.z;

    return c;
}

sensors_vec_t operator+=(sensors_vec_t a, sensors_vec_t b) {
    return a + b;
}

sensors_event_t operator+(sensors_event_t a, sensors_event_t b) {
    sensors_event_t c = a;
    c.acceleration = a.acceleration + b.acceleration;
    c.gyro = a.gyro + b.gyro;
    return c;
}

sensors_vec_t operator*(sensors_vec_t a, double b) {
    sensors_vec_t c = a;
    c.x *= b;
    c.y *= b;
    c.z *= b;
    return c;
}

sensors_event_t operator*(sensors_event_t a, double b) {
    sensors_event_t c = a;
    c.acceleration = a.acceleration * b;
    c.gyro = a.gyro * b;
    return c;
}

sensors_vec_t operator/(sensors_vec_t a, double b) {
    sensors_vec_t c = a;
    b = 1/b;
    c.x *= b;
    c.y *= b;
    c.z *= b;
    return c;
}

sensors_event_t operator/(sensors_event_t a, double b) {
    sensors_event_t c = a;
    b = 1/b;
    c.acceleration = a.acceleration * b;
    c.gyro = a.gyro * b;
    return c;
}

void applyTime(double diff, sensors_vec_t a){
    diff /= 1000; //measurements are in x/s, and diff is in ms
    a.x *= diff;
    a.y *= diff;
    a.z *= diff;
}

sensors_event_t mpu_calibrate_gyro(Adafruit_MPU6050 &mpu) {
    sensors_event_t res;
    for (int i = 0; i < COUNT; ++i){
        sensors_event_t temp;
        mpu.getGyroSensor()->getEvent(&temp);
        res = res + temp;
    }
    return res / COUNT;
}

void setupMPU(Adafruit_MPU6050 &mpu){
    // MPU INIT
    //  Try to initialize!
    while (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
        delay(1000);
    }

    Serial.println("MPU6050 Found! starting in 1 second...");

    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void gyro_signals(void) {
    Wire.beginTransmission(0x68);  // Start the communication with the MPU6050
    Wire.write(0x1A);              // Enable Low-pass filter
    Wire.write(0x05);
    Wire.endTransmission();  // End Transmission
    Wire.beginTransmission(0x68);
    Wire.write(0x1B);  // Change Gyro sensitivity
    Wire.write(0x8);
    Wire.endTransmission();
    Wire.beginTransmission(0x68);  // Access register storing Gyro data
    Wire.write(0x43);
    Wire.endTransmission();
    Wire.requestFrom(0x68, 6);  // Read the Gyro data
    Gyro_raw_X = Wire.read() << 8 | Wire.read();
    Gyro_raw_Y = Wire.read() << 8 | Wire.read();
    Gyro_raw_Z = Wire.read() << 8 | Wire.read();
    GyroX = (float)Gyro_raw_X / 131.0;
    GyroY = (float)Gyro_raw_Y / 131.0;
    GyroZ = (float)Gyro_raw_Z / 131.0;
}

void acc_signals(void) {
    Wire.beginTransmission(0x68);  // Start the communication with the MPU6050
    Wire.write(0x1A);              // Enable Low-pass filter
    Wire.write(0x05);              // Enable Low-pass filter - 10Hz-ish
    Wire.endTransmission();        // End Transmission
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);  // Access register storing Acc data
    Wire.endTransmission();
    Wire.requestFrom(0x68, 6);  // Read the Acc data
    Acc_raw_X = Wire.read() << 8 | Wire.read();
    Acc_raw_Y = Wire.read() << 8 | Wire.read();
    Acc_raw_Z = Wire.read() << 8 | Wire.read();
    AccX = (float)Acc_raw_X / 16384.0;
    AccY = (float)Acc_raw_Y / 16384.0;
    AccZ = (float)Acc_raw_Z / 16384.0;
}

float floatMap(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void drSonMPU() {  // MPU preparation and calibration
    Wire.setClock(100000);            // set wire clock to the max (400kHz)
    Wire.begin();           // Start the wire comunication
    delay(100);
    Wire.beginTransmission(0x68);  // Start the communication with the MPU6050
    Wire.write(0x6B);              // Start the gyro in power mode
    Wire.write(0x00);              // Set the requested starting register
    Wire.endTransmission();        // End the transmission
    // MPU6050 Gyro Calibration
    const int count = 1000;
    Serial.println("Begin MPU Gyro Calibration");
    for (i = 0; i < count; i++) {
        gyro_signals();
        GyroX_calib += GyroX;
        GyroY_calib += GyroY;
        GyroZ_calib += GyroZ;
    }
    GyroX_calib /= count;
    GyroY_calib /= count;
    GyroZ_calib /= count;
    // MPU6050 Acc Calibration
    // Serial.println("Begin MPU Acc  Calibration");
    // for (i = 0; i < count; i++) {
    //     acc_signals();
    //     AccX_calib += AccX;
    //     AccY_calib += AccY;
    //     AccZ_calib += AccZ;
    // }
    // For the calibration of the accelerometer, check what is the direction of the MPU. For me g is in the negative X-axis
    // Because the gravitational acceleration should not be part of the calibration
    // AccX_calib = AccX_calib / count;
    // AccY_calib = AccY_calib / count;
    // AccZ_calib = AccZ_calib / count;

    Serial.println("MPU calibrated. starting in 1 second...");
    delay(1000);
}

void updateMPU(){
    curr_time = micros();
    gyro_signals();  // Get gyro data GyroX, GyroY, GyroZ
    acc_signals();   // Get acc data AccX, AccY, AccZ

    GyroX -= GyroX_calib;
    GyroY -= GyroY_calib;
    GyroZ -= GyroZ_calib;
    // AccX -= AccX_calib;
    // AccY -= AccY_calib;
    // AccZ -= AccZ_calib;

    dt_mpu = (curr_time - prev_time_mpu);
    prev_time_mpu = curr_time;

    Acc_angle[1] = atan2(AccZ, sqrt(pow(AccY, 2) + pow(AccX, 2))) * rad_to_deg;
    Acc_angle[2] = atan2(AccY, sqrt(pow(AccX, 2) + pow(AccZ, 2))) * rad_to_deg;

    Gyro_angle[0] += GyroX * dt_mpu / 1000000;
    Gyro_angle[1] -= GyroY * dt_mpu / 1000000;
    Gyro_angle[2] -= GyroZ * dt_mpu / 1000000;

    Total_angle[0] = Gyro_angle[0];
    Total_angle[1] = .98 * (Total_angle[1] + GyroY * dt_mpu / 1000000) + .02 * Acc_angle[1];
    Total_angle[2] = .98 * (Total_angle[2] + GyroX * dt_mpu / 1000000) + .02 * Acc_angle[2];
}