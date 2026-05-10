/**
 * @file multi_system.hpp
 * @brief Multi-system orchestration functions for the CPR machine.
 *
 * This header file contains function declarations that coordinate behavior across
 * multiple subsystems (actuation, sensing, and HMI). These functions represent
 * higher-level operations that require synchronized interaction between different
 * hardware and software components to achieve complex machine behaviors.
 *
 * @date 2026-05-10
 * @version 1.0
 */

#ifndef MULTI_SYSTEM_HPP
#define MULTI_SYSTEM_HPP

#include "actuation.hpp"
#include "hmi.hpp"
#include "sensing.hpp"

/**
 * @brief Executes the mechanical zeroing sequence.
 *
 * This function coordinates the actuation of solenoids and error detection to perform the entire mechanical zeroing process.
 *
 * @return int PLACEHOLDER, will eventually return error state
 */
int mechanical_zeroing();

/**
 * @brief Executes the fine zeroing sequence.
 *
 * This function coordinates fine zeroing through motor velocity control and sensor feedback.
 *
 * @return int PLACEHOLDER, will eventually return error state
 */
int fine_zeroing();

/**
 * @brief Waits for the user to press the next button.
 *
 * This function monitors the HMI input system until the next button is pressed,
 * initiating a state change when the next button is required for said change.
 *
 * @return int PLACEHOLDER, will eventually return error state
 */
int wait_for_next_button();

#endif // MULTI_SYSTEM_HPP