/**
 * @file hmi.cpp
 * @brief Implementation of Human-Machine Interface functions for the CPR machine.
 *
 * This source file contains the implementations of human-machine interface operations
 * in the CPR machine.
 *
 * @date 2026-05-10
 * @version 1.0
 */

#include "hmi.hpp"

Error_Code init_screens() {
    return Error_Code::NORMAL_OPERATION;
}

bool set_screen_audio() {
    return true;
}

void enable_lasers() {
    return;
}

void disable_lasers() {
    return;
}

Error_Code enable_next_button() {
    return Error_Code::NORMAL_OPERATION;
}

Error_Code disable_next_button() {
    return Error_Code::NORMAL_OPERATION;
}

Error_Code enable_pause_button() {
    return Error_Code::NORMAL_OPERATION;
}

Error_Code disable_pause_button() {
    return Error_Code::NORMAL_OPERATION;
}

Error_Code read_buttons() {
    return Error_Code::NORMAL_OPERATION;
}