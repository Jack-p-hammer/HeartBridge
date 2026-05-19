/**
 * @file sensing.cpp
 * @brief Implementation of sensing functions for the CPR machine.
 *
 * This source file contains the implementations of sensing-related operations
 * in the CPR machine.
 *
 * @date 2026-05-10
 * @version 1.0
 */

#include "sensing.hpp"

Error_Code init_sensors() {
    // Setup I2C bus:
    
    
    // Sensors to initiatilize:
    // - IMU 
    // - Force sensor 
    // - Time of Flight sensor
    ////// The ones below are on the motor controller board
    // - Rotary encoder
    // - Battery level

    Adafruit_VL53L0X ToFSensor = Adafruit_VL53L0X();


    



    return Error_Code::NORMAL_OPERATION;
}

Error_Code battery_check() {
    return Error_Code::NORMAL_OPERATION;
}

Error_Code read_sensors_mech_zeroing() {
    return Error_Code::NORMAL_OPERATION;
}

Error_Code read_sensors_fine_zeroing() {
    return Error_Code::NORMAL_OPERATION;
}

Error_Code read_sensors_compressions() {
    return Error_Code::NORMAL_OPERATION;
}

Error_Code check_sensor_error() {
    return Error_Code::NORMAL_OPERATION;
}

Error_Code zero_chest_position() {
    return Error_Code::NORMAL_OPERATION;
}