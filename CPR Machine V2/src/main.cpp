/**
 * @file main.cpp
 * @brief Main entry point for the CPR machine state machine.
 *
 * This source file contains the main function that runs the state machine
 * for the CPR machine.
 *
 * @date 2026-05-10
 * @version 1.0
 */

#include <iostream>
#include "error_codes.hpp"
#include "actuation.hpp"
#include "hmi.hpp"
#include "sensing.hpp"
#include "multi_system.hpp"

// Enum of possible device states
enum CPR_Machine_State {
    STARTUP,
    UNFOLD_CUT_CLOTHES,
    ALIGNMENT,
    ZEROING_PREP,
    MECHANICAL_ZEROING,
    FINE_ZEROING,
    COMPRESSION_PREP,
    COMPRESSION,
    PAUSE,
    ABORT,
    KNEEL_FAILURE
};

Error_Code main(void) {
    Error_Code error_code = Error_Code::NORMAL_OPERATION;
    // Stay in loop unless something really fucked happens
    while (error_code != Error_Code::EXIT_UNKNOWN && error_code != Error_Code::ERROR_UNKNOWN_IMAGE) { 
        static CPR_Machine_State currentState = STARTUP; // Placeholder initial state, declared as static in loop to preserve scope

        // State-independent code, left empty for now
        if(false){
            // Nothing yet
        }

        // Determine state based on error codes
        switch(error_code) {
            case Error_Code::EXIT_UNKNOWN:
                currentState = ABORT; // Should never be hit, but if it is, abort for safety
                break;
            case Error_Code::ERROR_INIT_FAILURE:
                currentState = ABORT;
                break;
            case Error_Code::ERROR_SENSOR_FAILURE:
                currentState = ABORT;
                break;
            case Error_Code::ERROR_IMU_KNEEL_FAILURE:
                currentState = KNEEL_FAILURE;
                break;
            case Error_Code::ERROR_MECH_ZEROING_TIMEOUT:
                currentState = ABORT;
                break;
            case Error_Code::ERROR_FINE_ZEROING_FAILURE:
                currentState = ABORT;
                break;
            default:
                // Normal operation - state transitions should be determined by button presses and sensor readings, not error codes, so no state change here
                break;
        }

        switch (currentState) {
            case STARTUP:
                // Initialization code - runs once at startup, then transitions to next state
                // Initialize sensors, motor, and screens, then display 911 screen and wait for next button press to transition to next state
                error_code = init_sensors();
                if (error_code != Error_Code::NORMAL_OPERATION) {
                    std::cerr << "Error during sensor initialization: " << static_cast<int>(error_code) << std::endl;
                    break;
                }

                error_code = init_motor();
                if (error_code != Error_Code::NORMAL_OPERATION) { 
                    std::cerr << "Error during motor initialization: " << static_cast<int>(error_code) << std::endl;
                    break;
                }

                error_code = init_screens();
                if (error_code != Error_Code::NORMAL_OPERATION) {
                    std::cerr << "Error during screen initialization: " << static_cast<int>(error_code) << std::endl;
                    break;
                }

                error_code = battery_check();
                if (error_code != Error_Code::NORMAL_OPERATION) {
                    std::cerr << "Error during battery check: " << static_cast<int>(error_code) << std::endl;
                    break;
                }

                // If we got this far, init successful, Display 911 screen and wait for next button press
                // TODO: Implement set_screen_audio() to set 911 image and audio prompt
                if(wait_for_next_button()) {
                    currentState = UNFOLD_CUT_CLOTHES;
                }
                break;
            case UNFOLD_CUT_CLOTHES:
                // Display instructions to unfold and cut clothes, and wait for next button press to confirm completion
                // TODO: Implement set_screen_audio() to set unfold/cut clothes image and audio prompt
                if(wait_for_next_button()) {
                    currentState = ALIGNMENT;
                }
                break;
            case ALIGNMENT:
                // Enable lasers for user alignment, and wait for next button press to confirm alignment
                enable_lasers();
                // TODO: Implement set_screen_audio() to set alignment image and audio prompt
                if(wait_for_next_button()) {
                    disable_lasers();
                    currentState = ZEROING_PREP;
                }
                break;
            case ZEROING_PREP:
                // Display instructions to prepare for zeroing, and wait for next button press to begin zeroing process
                // TODO: Implement set_screen_audio() to set zeroing preparation image and audio prompt
                if(wait_for_next_button()) {
                    currentState = MECHANICAL_ZEROING;
                }
                break;
            case MECHANICAL_ZEROING:
                // Release egg, wait for force sensor to detect egg weight on chest, then lock egg and move to fine zeroing

                break;
            case FINE_ZEROING:
                // Move end effector down until force threshold reached, move to compression prep
                break;
            case COMPRESSION_PREP:
                // Display instructions to prepare for compressions, then go straight to compressions once audio finishes
                // TODO: Implement set_screen_audio() to set compression preparation image and audio prompt
                error_code = init_compressions();
                if(set_screen_audio()) {
                    currentState = COMPRESSION;
                }
                break;
            case COMPRESSION:
                // Perform compressions
                error_code = compressions();
                break;
            case PAUSE:
                // Pause button pressed, return to chest zero position and wait for next button press to return to compressions
                // TODO: Implement set_screen_audio() to set pause image and audio prompt
                stop_compressions();
                if(wait_for_next_button()) {
                    currentState = COMPRESSION_PREP;
                }
                break;
            case ABORT:
                // If we get here, display abort screen and halt operations until power cycle
                // TODO: Implement set_screen_audio() to set abort image and audio prompt
                break;
            case KNEEL_FAILURE:
                // IMU has been tripped, assume device is now out of position. 
                // Return to chest zero, display kneel failure screen, and wait for next button press to return to alignment state
                // TODO: Implement set_screen_audio() to set kneel failure image and audio prompt
                if(wait_for_next_button()) {
                    currentState = COMPRESSION_PREP;
                }
                break;
            // Error condition if state is not recognized - should never be hit if all states are handled properly
            default:
                error_code = Error_Code::EXIT_UNKNOWN;
                break;
        }
    }

    return error_code;
}