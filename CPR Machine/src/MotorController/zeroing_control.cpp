#include "zeroing_control.h"
#include <math.h>
#include <Arduino.h>
#include <ACAN2517FD.h>
#include <Moteus.h>
#include "control_scheme.h"
#include "sensors.h"
#include <assert.h>
#include "HMI_utils.h"

double const zeroPos = 1.72;//1.72;
double current_error_gain = 68.58;

double prev_command_gain = 1.002;
double prev_error_gain = -113.9;

double prev_prev_command_gain = -.00201;
double prev_prev_error_gain = 50.2;

const double extensionStrokeLimit = 0.0254*8; // 10 inches, in meters
long zeroing_start_time = 0;
const double zeroingVelocity = 0.04; // m/s

void initializeZeroing() {
    // Initialize outer loop
    nextSendMillis = millis();
    loopCount = 0;

    // Record zeroing start time
    zeroing_start_time = millis();
    // absLinearZero = read_linear_encoder();
    // absRotaryZero = read_rotary_encoder();
}

bool updateZeroing() {
    // We intend to send control frames every controller_period ms.
    const auto time = millis();

    // Only continue if it's time, may cause issues w/HMI integration when on one MCU
    if (nextSendMillis >= time) { return false; } // Return false because not zeroed yet

    nextSendMillis += controller_period;
    loopCount++;

    // Update sensor variables and error check
    // readSensors() updates global variables, so call inside if statement works 
    // as if outside the if
    // if(!readSensors()) {
    //     prevState = currentState;
    //     currentState = ABORT;
    //     currentGroupNum = 10;
    //     return false;
    // }
    DPRINT(">");
    DPRINT("Force:"); DPRINTLN(forceVal);
    // DPRINT(",");
    // DPRINT("Prev_State:"); DPRINT(prevState);
    // DPRINT(",");
    // DPRINT("STATE:"); DPRINTLN(currentState);

    // // Check for zeroing failure conditions
    // if(linearPos > extensionStrokeLimit) {
    //     prevState = currentState;
    //     currentState = ABORT;
    //     currentGroupNum = 10; // Set to abort group
    //     return false;
    // }

    // Check for successful zeroing
    // TODO: Refine zeroing setpoint to be weight of plunger-rack system
    

    // if(forceVal >= 35) {
    //     // Handle state change in main state machine, just return true for now

    //     zeroLinearEncoder();
    //     zeroRotaryEncoder();
    //     DPRINTLN("ZEROING COMPLETE");
    //     moteus.SetBrake(); // might have to move this
    //     moteus.SetStop();
    //     //currentGroupNum = 6;
    //     return true;
    // }

    // Send control command using position mode as a ramp in position,
    // which approximates constant velocity motion but goes through
    // the same (working) position-control path as compressions.
    //
    // The helper already computes a ramped position in meters from
    // the zeroing start time using `zeroingVelocity`:
    //   outputPos_m = zeroingVelocity * time_sec;
    // and clamps it to `extensionStrokeLimit`.
    // double zeroingSetpoint_m = computeZeroingSetpoint();
    // sendCommands(zeroingSetpoint_m, POSITION);

    //sendCommands(zeroingVelocity/(2*PI*pinionRadius), VELOCITY);
    sendCommands(zeroPos, ZEROING_POSITION);
    DPRINT("Zero Error: "); DPRINTLN(moteus.last_result().values.position - zeroPos);

     if(abs(moteus.last_result().values.position - zeroPos) < 0.05) {
        // Handle state change in main state machine, just return true for now

        zeroLinearEncoder();
        zeroRotaryEncoder();
        DPRINTLN("ZEROING COMPLETE");
        moteus.SetBrake(); // might have to move this
        moteus.SetStop();
        //currentGroupNum = 6;
        return true;
    }

    // Only print status every 25th cycle.
    if (loopCount % 10 == 0) {
        // printStatus(nextSendMillis);
                
        DPRINT(">");
        DPRINT("LINEAR POS:"); DPRINT(linearPos-linearZeroPos);
        DPRINT(",");
        DPRINT("PreviousState:"); DPRINT(prevState);
                DPRINT(",");
        DPRINT("STATE:"); DPRINTLN(currentState);
    }

    // No errors, return false because setpoint not found
    return false;
}

void returnToPreZeroingZero() {
    // For whatever reason, we need to go to our uncalibrated zero
    // If pre-calibration, this defaults to position on startup

    // We intend to send control frames every controller_period ms.
    const auto time = millis();
    if (nextSendMillis >= time) { return; }

    nextSendMillis += controller_period;
    loopCount++;

    readSensors();
    sendCommands(updateZeroingController(0), VELOCITY);

}
double updateZeroingController(double setpoint_m) {
  readSensors();

  // TODO: Update so that it references absolute 0, not zeroed zero

  // Take setpoint relative to linear zero
  //double error = (setpoint_m - linearZeroPos) - linearPos;
  double error = setpoint_m - rotaryPos*(2*PI)*pinionRadius;

  // See MATLAB file SimulinkSetup.mlx for controller in discrete TF form
  // THIS REQUIRES 10 ms CONTROLLER UPDATE PERIOD
  assert(controller_period == 10);

  // Different controller setup --> no chest in the way
  double torqueOutput = current_error_gain*error
                      + prev_command_gain*prevCommand + prev_error_gain*prevError 
                      + prev_prev_command_gain*prevPrevCommand + prev_prev_error_gain*prevPrevError; 
                      

  // Saturate the control effort
  torqueOutput = constrain(torqueOutput, -5, 5);

  prevPrevCommand = prevCommand;
  prevPrevError = prevError;

  prevCommand = torqueOutput;
  prevError = error;

  return torqueOutput;
}

double computeZeroingSetpoint() {
    // Send ramp input from starting position
    // Assume we start at zero position

    double time_sec = (millis() - zeroing_start_time) / 1000.0;
    double outputPos_m = zeroingVelocity * time_sec;

    // limit outputPos to max stroke
    if(outputPos_m > extensionStrokeLimit) {
        outputPos_m = extensionStrokeLimit;
    }
    return outputPos_m;
}