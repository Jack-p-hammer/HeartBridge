/**
 * @file error_codes.hpp
 * @brief Error and exit code definitions for the CPR machine.
 *
 * This header defines shared error and exit codes returned by CPR
 * machine subsystems and the main state machine.
 *
 * @date 2026-05-10
 * @version 1.0
 */

#ifndef ERROR_CODES_HPP
#define ERROR_CODES_HPP

/**
 * @brief Shared error and exit codes.
 *
 * This enum defines values for normal exit, error conditions, and
 * diagnostics used throughout the CPR machine firmware.
 */
enum class Error_Code {
    EXIT_UNKNOWN = 0,
    ERROR_INIT_FAILURE,
    ERROR_SENSOR_FAILURE,
    ERROR_IMU_KNEEL_FAILURE,
    ERROR_MECH_ZEROING_TIMEOUT,
    ERROR_FINE_ZEROING_FAILURE,
    NORMAL_OPERATION = 0x7F
};

#endif // ERROR_CODES_HPP
