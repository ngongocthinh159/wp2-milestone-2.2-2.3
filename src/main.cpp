#include <ACS712.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <HX711_ADC.h>
#include <PID_v1.h>
#include <Servo.h>
#include <Wire.h>
#include <kalman.h>
#include <loadcell.h>
#include <motor.h>
#include <mpu.h>
#include <utils.h>

Adafruit_MPU6050 mpu;

sensors_event_t acc, gyro, gyro_offset, sum_gyro;

// Define Variables we'll be connecting to
PID_Input leftInput, rightInput;

Servo leftMotor, rightMotor;

double P = 49, I = 6.0, D = 5.7;

double overlap = 0.00;

double START_SIGNAL = 6;

double pTunningStep = 0.5;
double iTunningStep = 0.5;
double dTunningStep = 0.1;
double overlapTunningStep = 0.01;
double minSignalTunningStep = 0.1;

// maximum rate of change of motor output per second
double maxChangePerSecond = 5000;

PID leftPID(&leftInput.Input, &leftInput.Output, &leftInput.Setpoint, P, I, D, P_ON_E, DIRECT);

PID rightPID(&rightInput.Input, &rightInput.Output, &rightInput.Setpoint, P, I, D, P_ON_E, REVERSE);

HX711_ADC LoadCell(HX711_dout, HX711_sck);

void setup() {
    // initialize serial communications at 9600 bps:
    Serial.begin(9600);
    // wait for serial communication is ready
    // this requires serial communication to be open for the program to run
    while (!Serial)
        ;

    setupMotor(leftMotor, LEFT_MOTOR_OUT_PIN);
    setupMotor(rightMotor, RIGHT_MOTOR_OUT_PIN);

    leftPID.SetMode(AUTOMATIC);
    // set the output of our PID to match with the range of our esc's output range
    // this range is calibrated during manual configuration (LOADCELL_CALIBRATED = 0)
    leftPID.SetOutputLimits(MIN_SIGNAL + SIGNAL_RANGE * START_SIGNAL / 100, MIN_SIGNAL + SIGNAL_RANGE * 1);
    leftPID.SetSampleTime(1);

    rightPID.SetMode(AUTOMATIC);
    // set the output of our PID to match with the range of our esc's output range
    // this range is calibrated during manual configuration (LOADCELL_CALIBRATED = 0)
    rightPID.SetOutputLimits(MIN_SIGNAL + SIGNAL_RANGE * START_SIGNAL / 100, MIN_SIGNAL + SIGNAL_RANGE * 1);
    rightPID.SetSampleTime(1);

    pinMode(A4, INPUT_PULLUP);
    pinMode(A5, INPUT_PULLUP);

    drSonMPU();

    loadcell_setup(LoadCell);
}

double deg_kalman_z = 0;
double deg_err_z = initialEstimateErr;

double deg_kalman_y = 0;
double deg_err_y = initialEstimateErr;

bool startMotor = false;
bool manualMotor = false;

int32_t lastTime = 0;

double prevLeftOutput = MIN_SIGNAL;
double prevRightOutput = MIN_SIGNAL;

//************************************************t*** LOOP FUNCTION ***************************************************
void loop() {
    /* Get new sensor events with the readings */
    char c = readSerial();
    if (c == 't') {
        drSonMPU();
        // gyro_offset = mpu_calibrate_gyro(mpu);
        sum_gyro = sensors_event_t();

        deg_kalman_z = 0;
        deg_err_z = initialEstimateErr;

        deg_kalman_y = 0;
        deg_err_y = initialEstimateErr;
    } else if (c == 's') {
        if (manualMotor) {
            manualMotor = false;
        } else {
            startMotor = !startMotor;
        }
    } else if (c == 'e') {
        Serial.println("Turn off motor and hit c");
        while (readSerial() != 'c')
            ;
        Serial.println("Starting calibration");
        Serial.println("Turn on power source for 2 seconds, then turn power source off, and hit c");
        while (readSerial() != 'c') {
            leftMotor.writeMicroseconds(MAX_SIGNAL);
        }
        Serial.println("setting left motor...");
        leftMotor.writeMicroseconds(MIN_SIGNAL);
        delay(1000);
        Serial.println("Calibration done");
        delay(500);

    } else if (c == 'r') {
        Serial.println("Turn off motor and hit c");
        while (readSerial() != 'c')
            ;
        Serial.println("Starting calibration");
        Serial.println("Turn on power source for 2 seconds, then turn power source off, and hit c");
        while (readSerial() != 'c') {
            rightMotor.writeMicroseconds(MAX_SIGNAL);
        }
        Serial.println("setting left motor...");
        rightMotor.writeMicroseconds(MIN_SIGNAL);
        delay(1000);
        Serial.println("Calibration done");
        delay(500);
    } else if (c == 'g') {
        P = fabs(P + pTunningStep);
        leftPID.SetTunings(P, I, D);
        rightPID.SetTunings(P, I, D);
    } else if (c == 'v') {
        P = fabs(P - pTunningStep);
        leftPID.SetTunings(P, I, D);
        rightPID.SetTunings(P, I, D);
    } else if (c == 'j') {
        D = fabs(D + dTunningStep);
        leftPID.SetTunings(P, I, D);
        rightPID.SetTunings(P, I, D);
    } else if (c == 'n') {
        D = fabs(D - dTunningStep);
        leftPID.SetTunings(P, I, D);
        rightPID.SetTunings(P, I, D);
    } else if (c == 'h') {
        I = fabs(I + iTunningStep);
        leftPID.SetTunings(P, I, D);
        rightPID.SetTunings(P, I, D);
    } else if (c == 'b') {
        I = fabs(I - iTunningStep);
        leftPID.SetTunings(P, I, D);
        rightPID.SetTunings(P, I, D);
        // }
    } else if (c == 'i') {
        manualMotor = true;
        leftInput.Output = MIN_SIGNAL + SIGNAL_RANGE * 0.5;
    } else if (c == 'o') {
        manualMotor = true;
        rightInput.Output = MIN_SIGNAL + SIGNAL_RANGE * 0.5;
    } else if (c == 'p') {
        manualMotor = true;
        leftInput.Output = MIN_SIGNAL + SIGNAL_RANGE * 0.5;
        rightInput.Output = MIN_SIGNAL + SIGNAL_RANGE * 0.5;
    } else if (c == 'd') {
        START_SIGNAL = abs(START_SIGNAL + minSignalTunningStep);
        leftPID.SetOutputLimits(MIN_SIGNAL + SIGNAL_RANGE * START_SIGNAL / 100, MIN_SIGNAL + SIGNAL_RANGE * 1);
        rightPID.SetOutputLimits(MIN_SIGNAL + SIGNAL_RANGE * START_SIGNAL / 100, MIN_SIGNAL + SIGNAL_RANGE * 1);
    } else if (c == 'x') {
        START_SIGNAL = abs(START_SIGNAL - minSignalTunningStep);
        leftPID.SetOutputLimits(MIN_SIGNAL + SIGNAL_RANGE * START_SIGNAL / 100, MIN_SIGNAL + SIGNAL_RANGE * 1);
        rightPID.SetOutputLimits(MIN_SIGNAL + SIGNAL_RANGE * START_SIGNAL / 100, MIN_SIGNAL + SIGNAL_RANGE * 1);
    } else if (c == 'f') {
        overlap = abs(overlap + overlapTunningStep);
    } else if (c == 'c') {
        overlap = abs(overlap - overlapTunningStep);
    } else if (c == 'l') {
        leftMotor.writeMicroseconds(MIN_SIGNAL);
        rightMotor.writeMicroseconds(MIN_SIGNAL);
        delay(2000);

        LoadCell.tareNoDelay();  // Tare load cell non-blocking
    } else if (c == 'w'){
        leftInput.Setpoint++;
        rightInput.Setpoint++;
    } else if (c == 'q'){
        leftInput.Setpoint--;
        rightInput.Setpoint--;
    }

    // Update MPU
    updateMPU();

    // after getEvent, before any calculations
    // double timeDif = 1.0 * (gyro.timestamp - lastTime) / 1000.0;
    double timeDif = dt_mpu / 1e6;

    acc.acceleration.x = AccX;
    acc.acceleration.y = AccY;
    acc.acceleration.z = AccZ;

    /* gyro */
    // dr son already account for offset
    //  acc = acc - acc_offset;
    //  gyro = gyro - gyro_offset;

    gyro.gyro.x = GyroX;
    gyro.gyro.y = GyroY;
    gyro.gyro.z = GyroZ;

    sum_gyro = sum_gyro + gyro * timeDif;

    // gyro angle is already in sum
    //  double deg_gyro_x = sum_gyro.gyro.x;
    //  double deg_gyro_y = -sum_gyro.gyro.y;
    // double deg_gyro_z = sum_gyro.gyro.z;
    // double deg_gyro_x = Gyro_angle[0];
    // double deg_gyro_y = -Gyro_angle[1];
    // double deg_gyro_z = Gyro_angle[2];

    double deg_acc_y = atan(acc.acceleration.z / sqrt(sqr(acc.acceleration.y) + sqr(acc.acceleration.x))) / PI * 180;
    double deg_acc_z = atan(-acc.acceleration.y / sqrt(sqr(acc.acceleration.x) + sqr(acc.acceleration.z))) / PI * 180;

    kalman_1d(deg_kalman_z, deg_err_z, gyro.gyro.z, deg_acc_z, timeDif);
    kalman_1d(deg_kalman_y, deg_err_y, gyro.gyro.y, deg_acc_y, timeDif);

    // double deg_kalman_z_temp, deg_kalman_y_temp;

    // kalman_1d(deg_kalman_z_temp, deg_err_z, gyro.gyro.z, deg_acc_z, timeDif);
    // kalman_1d(deg_kalman_y_temp, deg_err_y, gyro.gyro.y, deg_acc_y, timeDif);

    // if (fabs(deg_kalman_z_temp - deg_kalman_z) > 0.1) {
    //     deg_kalman_z = deg_kalman_z_temp;
    // }

    // if (fabs(deg_kalman_y_temp - deg_kalman_y) > 0.1) {
    //     deg_kalman_y = deg_kalman_y_temp;
    // }
    // deg_kalman_z = -Total_angle[2];

    leftInput.Input = deg_kalman_z;
    rightInput.Input = deg_kalman_z;
    /* gyro */

    /* update last timestamp*/
    // after all calculations
    // lastTime = gyro.timestamp;

    // only run the motor is tare operation is done and start motor is true
    if (!startMotor && !manualMotor) {
        leftInput.Input = 0;
        leftInput.Output = MIN_SIGNAL;
        leftPID.SetMode(MANUAL);
        leftMotor.writeMicroseconds(MIN_SIGNAL);

        rightInput.Input = 0;
        rightInput.Output = MIN_SIGNAL;
        rightPID.SetMode(MANUAL);
        rightMotor.writeMicroseconds(MIN_SIGNAL);
    } else if (manualMotor) {
        leftPID.SetMode(MANUAL);
        leftMotor.writeMicroseconds(leftInput.Output);

        rightPID.SetMode(MANUAL);
        rightMotor.writeMicroseconds(rightInput.Output);
    } else {
        leftPID.SetMode(AUTOMATIC);
        leftPID.Compute();

        if (leftInput.Output - prevLeftOutput > maxChangePerSecond * leftPID.getTimeChange() / 1000) {
            leftInput.Output = prevLeftOutput + maxChangePerSecond * leftPID.getTimeChange() / 1000;
        }

        leftMotor.writeMicroseconds(leftInput.Output);

        rightPID.SetMode(AUTOMATIC);
        rightPID.Compute();

        if (rightInput.Output - prevRightOutput > maxChangePerSecond * rightPID.getTimeChange() / 1000) {
            rightInput.Output = prevRightOutput + maxChangePerSecond * rightPID.getTimeChange() / 1000;
        }

        rightMotor.writeMicroseconds(rightInput.Output);
    }

    prevLeftOutput = leftInput.Output;
    prevRightOutput = rightInput.Output;

    // Get load cell data
    static boolean newDataReady = 0;
    if (LoadCell.update()) newDataReady = true;

    /* Print out the values */
    if (newDataReady) {
        float loadCellData = LoadCell.getData();
        Serial.printf("time: %d\n", millis());
        Serial.printf("deg_kalman_y: %7.2f\n", deg_kalman_y);
        Serial.printf("deg_kalman_z: %7.2f\n", deg_kalman_z);
        Serial.printf("leftOutput: %7f\nrightOutput: %7f\n", leftInput.Output, rightInput.Output);
        Serial.printf("P: %7f\nI: %7f\nD: %7f\n", P, I, D);
        Serial.printf("overlap: %7f\n", overlap);
        Serial.printf("startSignal: %7f\n", START_SIGNAL);
        Serial.printf("loadCell: %7f\n", loadCellData);
        Serial.printf("Setpoint: %7f\n", leftInput.Setpoint);
        Serial.printf("\r\n");
        newDataReady = 0;
    }

    // delay(10);
}