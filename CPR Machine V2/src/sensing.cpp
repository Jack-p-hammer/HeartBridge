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
#include <pigpio.h>
#include <cstdint>
#include <iostream>

// I2C addresses
static const int VL53L0X_ADDR = 0x29;
static const int BNO085_ADDR  = 0x4A;

// pigpio I2C handles
static int vl53l0x_handle = -1;
static int bno085_handle  = -1;

// --- I2C helpers ---

static bool i2c_write_reg(int handle, uint8_t reg, uint8_t val) {
    return i2cWriteByteData(handle, reg, val) == 0;
}

static int i2c_read_reg(int handle, uint8_t reg) {
    return i2cReadByteData(handle, reg); // returns value or negative on error
}

// --- VL53L0X init ---

static bool init_vl53l0x() {
    vl53l0x_handle = i2cOpen(1, VL53L0X_ADDR, 0);
    if (vl53l0x_handle < 0) {
        std::cerr << "VL53L0X not found at address 0x29" << std::endl;
        return false;
    }

    // Read model ID register - should return 0xEE if device is present
    int model_id = i2c_read_reg(vl53l0x_handle, 0xC0);
    if (model_id != 0xEE) {
        std::cerr << "VL53L0X model ID check failed" << std::endl;
        return false;
    }

    // Basic init sequence to prepare device for ranging
    i2c_write_reg(vl53l0x_handle, 0x88, 0x00);
    i2c_write_reg(vl53l0x_handle, 0x80, 0x01);
    i2c_write_reg(vl53l0x_handle, 0xFF, 0x01);
    i2c_write_reg(vl53l0x_handle, 0x00, 0x00);
    i2c_write_reg(vl53l0x_handle, 0xFF, 0x00);
    i2c_write_reg(vl53l0x_handle, 0x80, 0x00);

    std::cout << "VL53L0X initialized" << std::endl;
    return true;
}

// --- BNO085 init ---

static bool init_bno085() {
    bno085_handle = i2cOpen(1, BNO085_ADDR, 0);
    if (bno085_handle < 0) {
        std::cerr << "BNO085 not found at address 0x4A" << std::endl;
        return false;
    }

    // BNO085 sends a 4-byte SHTP header on startup - read it to confirm presence
    char buf[4];
    if (i2cReadDevice(bno085_handle, buf, 4) < 0) {
        std::cerr << "BNO085 did not respond" << std::endl;
        return false;
    }

    // TODO: Enable accelerometer report via SHTP for kneel detection

    std::cout << "BNO085 initialized" << std::endl;
    return true;
}

// --- Public API ---

Error_Code init_sensors() {
    // Initialize pigpio
    if (gpioInitialise() < 0) {
        std::cerr << "Failed to initialize pigpio" << std::endl;
        return Error_Code::ERROR_INIT_FAILURE;
    }

    if (!init_vl53l0x()) return Error_Code::ERROR_INIT_FAILURE;
    if (!init_bno085())  return Error_Code::ERROR_INIT_FAILURE;

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
