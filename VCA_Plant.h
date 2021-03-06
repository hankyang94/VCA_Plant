//
// Created by Hank on 7/11/2018.
//

#ifndef VCA_PLANT_H
#define VCA_PLANT_H

#include "Arduino.h"

class VCA_Plant {
public:
    VCA_Plant(void);     // default constructor

    VCA_Plant(int motor_A_pwm_1, int motor_A_pwm_2, int motor_B_pwm_1, int motor_B_pwm_2,
              float motor_A_amplitude, int motor_A_frequency, float motor_B_amplitude, int motor_B_frequency,
              int motor_A_hall_pin, int motor_B_hall_pin, const float voltage_per_bit, const int drive_read_micros);    // constructor with parameters

    ~VCA_Plant(void);    // destructor

    void DriveMotorADuty(float duty);  // drive motor A with a duty cycle

    void DriveMotorBDuty(float duty);  // drive motor B with a duty cycle

    void StopMotorA(void);  // stop sending pwm, stop the motor

    void StopMotorB(void);  // stop sending PWM, stop the motor

    void StopMotor(void);  // stop both motors

    int ReadMotorAPositionBit();     // read the senor data in raw bits

    float ReadMotorAPositionVoltage();  // read the sensor data in voltage

    float ReadMotorAPositionMM();    // read the sensor data in mm

    float MotorACalibration(int bit);  // calibration result for motor A

    int ReadMotorBPositionBit();     // read the senor data in raw bits

    float ReadMotorBPositionVoltage();  // read the sensor data in voltage

    float ReadMotorBPositionMM();    // read the sensor data in mm

    float MotorBCalibration(int bit);   // calibration result for motor B


    void DriveMotorASin(float motor_A_max_duty,  // maximum duty cycle
                        int motor_A_frequency,   // vibration frequency
                        int num_cycles,          // vibration cycles
                        int loop_period);        // period of control loop, 1KHz, 1000us



private:
    int _motor_A_pwm_1;           // Arduino pin number for A PWM
    int _motor_A_pwm_2;           // Arduino pin number for A DIR
    int _motor_B_pwm_1;           // Arduino pin number for B PWM
    int _motor_B_pwm_2;           // Arduino pin number for B DIR
    float _motor_A_amplitude;       // motor A vibration amplitude
    int _motor_A_frequency;          // motor A vibration frequency
    float _motor_B_amplitude;        // motor B vibration amplitude, in mm
    int _motor_B_frequency;          // motor B vibration frequency, in HZ
//    int _motor_A_position_bit;       // motor A position feedback from hall effect sensor+AD620, raw bits
    int _motor_A_hall_pin;             // Arduino pin number for motor A hall effect sensor
    int _motor_B_hall_pin;             // Arduino pin number for motor B hall effect sensor
    const float _voltage_per_bit;    // scale for converting bits to voltage
//    double _motor_A_position_voltage;   // raw bits converted to voltage
    const int _drive_read_micros;          // Teensy drive and read microseconds in total

};



#endif //ARDUIVCA_ARDUINO_VCA_PLANT_H

