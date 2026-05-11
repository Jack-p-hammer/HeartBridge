/**
 * @file hmi.hpp
 * @brief Human-Machine Interface functions for the CPR machine.
 *
 * This header file contains function declarations for human-machine interface operations
 * in the CPR machine. These functions handle screen display, audio prompts, button inputs,
 * and the laser alignment system.
 *
 * @date 2026-05-10
 * @version 1.0
 */

#ifndef HMI_HPP
#define HMI_HPP

#include "error_codes.hpp"

/**
 * @brief Initializes the display screens.
 *
 * This function initializes all displays and prepares the screens for rendering.
 *
 * @return Error_Code
 */
Error_Code init_screens();

/**
 * @brief Sets screen instruction image and audio prompt.
 *
 * This function sets the instruction image to be displayed on the screen and plays the corresponding audio prompt. This is called repeatedly in each state, and loops the audio instruction until a different audio instruction is passed into the function.
 *
 * @return bool - True if autio finished playing once, False if not
 */
bool set_screen_audio();

/**
 * @brief Enables laser indicators.
 *
 * This function enables the laser diodes for user visual alignment, and performs PWM on the diodes (if we decide its necessary). Repeated calls within a loop run the PWM process.
 *
 * @return Nothing
 */
void enable_lasers();

/**
 * @brief Disables laser indicators.
 *
 * This function deactivates laser visual indicators.
 *
 * @return Nothing
 */
void disable_lasers();

/**
 * @brief Enables the next button.
 *
 * This function enables the 'next' button via enabling its built-in LED, as a result of the LED and button sharing a ground connection. This prevents accidental presses hardware-side.
 *
 * @return Error_Code
 */
Error_Code enable_next_button();

/**
 * @brief Disables the next button.
 *
 * This function disables the 'next' button via disabling its built-in LED, as a result of the LED and button sharing a ground connection. This prevents accidental presses hardware-side.
 *
 * @return Error_Code
 */
Error_Code disable_next_button();

/**
 * @brief Enables the pause button.
 *
 * This function enables the 'pause' button via enabling its built-in LED, as a result of the LED and button sharing a ground connection. This prevents accidental presses hardware-side.
 *
 * @return Error_Code
 */
Error_Code enable_pause_button();

/**
 * @brief Disables the pause button.
 *
 * This function disables the 'pause' button via disabling its built-in LED, as a result of the LED and button sharing a ground connection. This prevents accidental presses hardware-side.
 *
 * @return Error_Code
 */
Error_Code disable_pause_button();

/**
 * @brief Reads button input states.
 *
 * This function returns the current states of the 'next' and 'pause' buttons.
 *
 * @return Error_Code
 */
Error_Code read_buttons();

#endif // HMI_HPP