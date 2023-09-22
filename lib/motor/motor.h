#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include <Servo.h>

#define MAX_SIGNAL 2000
#define MIN_SIGNAL 1000
#define SIGNAL_RANGE ((MAX_SIGNAL) - (MIN_SIGNAL))

// set to 0 to do manual configuration, set to 1 to start the program with no manual configuration
#define MOTOR_CALIBRATED 1

struct PID_Input {
    double Setpoint;
    double Input;
    double Output;
};

extern const int LEFT_MOTOR_OUT_PIN;
extern const int RIGHT_MOTOR_OUT_PIN;

// function to calibrate the motor
void setupMotor(Servo& m, int motorOutPin);
void calibrateMotor(Servo& m);

#endif