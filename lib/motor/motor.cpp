#include <Arduino.h>
#include <Servo.h>
#include <motor.h>

const int LEFT_MOTOR_OUT_PIN = A8;
const int RIGHT_MOTOR_OUT_PIN = A9;

void setupMotor(Servo& motor, int motorOutPin) {

    // attach motor
    motor.attach(motorOutPin);

    // write 0 to motor
    motor.writeMicroseconds(MIN_SIGNAL);
    
    //done with setup

    //calibration time

    //return if we dont need to calibrate
    if (MOTOR_CALIBRATED) { 
        return;
    }

    while (Serial.available()) {
        Serial.read();
    }

    Serial.println("This program will start the ESC.");

    motor.attach(motorOutPin);

    motor.writeMicroseconds(MAX_SIGNAL);

    Serial.println("Turn off power source for 2 seconds, then turn power source on, and press any key to continue");

    while (!Serial.available());
    Serial.read();

    motor.writeMicroseconds(MIN_SIGNAL);
    delay(2000);

    Serial.println("The ESC is calibrated");
}