/**
 * @file actuation.hpp
 * @brief Actuation-related functions for the CPR machine.
 *
 * This header file contains function declarations for actuation-related operations
 * in the CPR machine. These functions are intended to be used directly in the main
 * state machine loop and handle the actuation of the end effector and solenoids
 * based on the current machine and error state.
 *
 * @date 2026-05-10
 * @version 1.0
 */

#ifndef ACTUATION_HPP
#define ACTUATION_HPP

#include "error_codes.hpp"

/**
 * @brief Initializes the motor for actuation.
 *
 * This function initializes the motor driver and its CAN communication. It should be called in the setup state.
 *
 * @return Error_Code
 */
Error_Code init_motor();

/**
 * @brief Engages solenoids to lock the egg.
 *
 * This function engages the solenoids to lock the egg, ideally raising it off the chest slightly to allow for fine zeroing.
 *
 * @return Error_Code
 */
Error_Code lock_egg();

/**
 * @brief Disengages solenoids to unlock the egg.
 *
 * This function disengages the solenoids to unlock the egg, allowing gravity to pull it down onto the chest for compressions.
 *
 * @return Error_Code
 */
Error_Code unlock_egg();

/**
 * @brief Runs the fine zeroing process.
 *
 * This function sends the predetermined zeroing velocity setpoint to the motor. Returned error state is based on whether fine zeroing range of motion has been exceeded, which would indicate a failure in the fine zeroing process and trigger the abort state. Force sensor readings are handled elsewhere.
 *
 * @return Error_Code
 */
Error_Code fine_zeroing();

/**
 * @brief Stops the fine zeroing process.
 *
 * This function sends a zero velocity setpoint and engages braking to hold the end effector in the zeroed position.
 *
 * @return Error_Code
 */
Error_Code stop_fine_zeroing();

/**
 * @brief Initializes the compression process
 *
 * This function changes the motor control mode to position control, swaps the associated gains, and re-enables the brake command in case it was lost. 
 *
 * @return Error_Code
 */
Error_Code init_compressions();

/**
 * @brief Starts compressions.
 *
 * This function runs the compression cycle. The position control setpoint is controlled withing this function based on time since compressions began. TODO: Decide when/where to record compression start time
 *
 * @return Error_Code
 */
Error_Code compressions();

/**
 * @brief Stops compressions.
 *
 * This function halts the compression cycle. This will return the end effector to the zeroed position and initiate a state change to the paused state.
 *
 * @return Error_Code
 */
Error_Code stop_compressions();

#endif // ACTUATION_HPP