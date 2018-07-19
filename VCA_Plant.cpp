//
// Created by Hank on 7/11/2018.
//


#include <math.h>
#include "VCA_Plant.h"


VCA_Plant::VCA_Plant(void)
        : VCA_Plant(35,     // pin 35 for pwm A 1
                          36,     // pin 36 for pwm A 2, direction pin
                          1.0,    // default amplitude +- 1 mm
                          20,     // default frequency 20Hz
                          33,     // pin 33 for reading hall effect sensor and AD620
                          7.6294e-5,  // resolution is 5V / 2^16, using 16 bit ADC
                          22      // takes about 22 micros to drive and read, print in console
                            ){}

VCA_Plant::VCA_Plant(int motor_A_pwm_1, int motor_A_pwm_2, float motor_A_amplitude, int motor_A_frequency,
                                 int motor_A_hall_pin, const float voltage_per_bit, const int drive_read_micros) :
                                  _motor_A_pwm_1(motor_A_pwm_1),
                                  _motor_A_pwm_2(motor_A_pwm_2),
                                  _motor_A_amplitude(motor_A_amplitude),
                                  _motor_A_frequency(motor_A_frequency),
                                  _motor_A_hall_pin(motor_A_hall_pin),
                                  _voltage_per_bit(voltage_per_bit),
                                  _drive_read_micros(drive_read_micros){
    analogReadResolution(16);
    // change the default PWM frequency to eliminate PWM switching noise
    analogWriteFrequency(motor_A_pwm_1, 30000);
    analogWriteFrequency(motor_A_pwm_2, 30000);
//    pinMode(motor_A_pwm_1, OUTPUT);
//    pinMode(motor_A_pwm_2, OUTPUT);
}

VCA_Plant::~VCA_Plant(void){}

void VCA_Plant::DriveMotorADuty(float duty) {
    int duty_bit = int(duty*256);
//    Serial.println(duty_bit);
    //////////////// use one-channel MAX14870 motor driver //////////////////////
    if (duty > 0) { // forward motor motion, PIN 1 PWM, PIN 2 HIGH
        analogWrite(_motor_A_pwm_1, duty_bit);
        digitalWrite(_motor_A_pwm_2, HIGH);
    } else if (duty < 0) { // reverse motor motion, PIN 1 PWM, PIN 2 LOW
        analogWrite(_motor_A_pwm_1, -duty_bit);
        digitalWrite(_motor_A_pwm_2, LOW);
    } else {}

    /* /////////// use two-channel DRV8833 motor driver /////////////////
    if (duty > 0) {  // forward motor motion slow decay, pin 1 HIGH, pin 2 PWM
        // slow decay
//        digitalWrite(_motor_A_pwm_1, HIGH);
//        analogWrite(_motor_A_pwm_1, 255);
//        analogWrite(_motor_A_pwm_2, duty_bit);

//        // fast decay
        analogWrite(_motor_A_pwm_1, duty_bit);
        analogWrite(_motor_A_pwm_2, 0);
    } else if (duty < 0) {  // reverse motor motion slow decay, pin 1 PWM, pin 2 1
        // slow decay
//        analogWrite(_motor_A_pwm_1, -duty_bit);
//        analogWrite(_motor_A_pwm_2, 255);
//        digitalWrite(_motor_A_pwm_2, HIGH);

        // fast decay
        analogWrite(_motor_A_pwm_1, 0);
        analogWrite(_motor_A_pwm_2, -duty_bit);
    } else {}
     */
    Serial1.println(duty_bit);
}

void VCA_Plant::StopMotorA() {  // stop motor, pin 1 and pin 2 both low
    analogWrite(_motor_A_pwm_1, 0);
    analogWrite(_motor_A_pwm_2, 0);
}

int VCA_Plant::ReadMotorAPositionBit() {
    int position = analogRead(_motor_A_hall_pin);
    Serial1.println(position);
    return position;
}

float VCA_Plant::ReadMotorAPositionVoltage() {
    int bit = this->ReadMotorAPositionBit();
    return bit * _voltage_per_bit;
}

void VCA_Plant::DriveMotorASin(float motor_A_max_duty, int motor_A_frequency, int num_cycles, int loop_period) {
    _motor_A_frequency = motor_A_frequency;
    int num_loops = int(1.0/float(motor_A_frequency)*1000000) / loop_period * num_cycles;
    int position_trajectory = 0;
    int timer = 0;
    int start_time = micros();
    for (int i=0; i<num_loops; i++) {
        timer = micros() - start_time;
        this->DriveMotorADuty(motor_A_max_duty * sin(2.0*M_PI*motor_A_frequency*(timer/1e6)));   // send out PWM
        position_trajectory = this->ReadMotorAPositionBit();             // read the sensor position
        while (micros() - timer - start_time < loop_period);  // wait for timer to increment, actually driving the VCA
    }
}
