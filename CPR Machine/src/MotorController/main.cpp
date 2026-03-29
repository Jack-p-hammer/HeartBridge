#include <Arduino.h>
#include "control_scheme.h"
#include "compression_control.h"
#include "zeroing_control.h"
#include "sensors.h"
#include "HMI_utils.h"
#include "interrupt_control.h"
#include <Moteus.h>
#include <ACAN2517FD.h>

long prepTimer = millis();
static bool firstRun = true;
uint32_t nextPrintMillis = 0;
int currentGroupNum = 0;

void setup() {
  Serial.begin(115200);
  // TODO: Remove this delay for M6
  //while (!Serial) {}
 
  // Do everything that needs to occur on power up
  initializeMotor();
  initializeSensors();
  HMI_util_setup();
  // TODO: Does anything else need to happen here?
  nextPrintMillis = millis();
  loopCount = 0;
}

void loop() {
  // Main State Machine
  // State changes into failure states handled by error detection code
  // State changes into success states handled in main switch case
  // FIXME: if this turns out to be a bad idea
  // if(currentState != ABORT) {
  //   DPRINT(currentState); DPRINT(" | "); DPRINTLN(linearPos);
  // }
  // long timer = millis();
    // Get current time once for all buttons
  GreenNow = millis();
  PauseNow = millis();

  const auto time = millis();
    if (nextPrintMillis >= time) { return; }

    nextPrintMillis += controller_period;
    loopCount++;

    // Only print our status every 25th cycle.
    if (loopCount % 25 == 0) {
        printStatus(nextPrintMillis);
        DPRINT(">");
        DPRINT("State:"); DPRINT(currentState);
        DPRINT(",");
        DPRINT("PrevState:"); DPRINTLN(prevState);  
        logBatteryData();
    }
    

  
  
  // DPRINT(", Zero:"); DPRINT(linearZeroPos*39.37);
  // DPRINT(", Setpoint:"); DPRINT((computeCompressionSetpoint()+ linearZeroPos)*39.37);
  // DPRINT(", LinearPos:"); DPRINT(linearPos*39.37);

 
  if(firstRun){
    showCurrentFrameAndAudio(currentGroupNum);
    firstRun = false;
  } else if (prevState != currentState){
    showCurrentFrameAndAudio(currentGroupNum);
  }

  

  switch (currentState) {
    case START_UP_BATTERY: 

      //currentGroupNum = 0; 
      //showScreen(startUpBmpFile);
      //playAudio(startUpWavFile);

      // This might cut off audio, we will have to test
       if(!verifyBatteryPercentage()) {
        // Handle low battery scenario
        prevState = currentState;
        currentState = ABORT;
        currentGroupNum = 9;
       }
      // } else {
      //   // Battery good, move to startup
      //   prevState = currentState;
      //   currentState = UNFOLD_EXPOSE;
      //   currentGroupNum = 1;
      // }
     

      if(nextButtonLoop()) {
        // User has pressed start, get started
        prevState = currentState;
        currentState = UNFOLD_EXPOSE;
        currentGroupNum = 1;
      }

      // Now that one loop has passed, update prevState
      if(currentState == START_UP_BATTERY) {
        prevState = currentState;
        // 
      }

      break;

    //case BATTERY_CHECK:
      //currentGroupNum = 0;
      
      // Immediately check battery state
     
      
      // Now that one loop has passed, update prevState
      // if(currentState == BATTERY_CHECK) {
      //   prevState = currentState;
      // }

     // break;

    case UNFOLD_EXPOSE: 
      // currentGroupNum = 1;
      //showScreen(cutClothingBmpFile);
      //playAudio(cutClothingWavFile);

      if(nextButtonLoop()) {
        // User has pressed start, get started
        prevState = currentState;
        currentState = ALIGNMENT;
        currentGroupNum = 2;
      }

      // Now that one loop has passed, update prevState
      if(currentState == UNFOLD_EXPOSE) {
        prevState = currentState;
       
      }

      break;
    
    // case CUT_CLOTHING:  
    //   // currentGroupNum = 2;
    //   // showScreen(unfoldBmpFile);
    //   // playAudio(unfoldWavFile);

    //   if(nextButtonLoop()) {
    //     // User has pressed start, get started
    //     prevState = currentState;
    //     currentState = ALIGNMENT;
    //     currentGroupNum = 3;
    //   }

    //   // Now that one loop has passed, update prevState
    //   if(currentState == CUT_CLOTHING) {
    //     prevState = currentState;
       
    //   }

    //   break;

    case ALIGNMENT:
      //currentGroupNum = 3;
      // showScreen(alignmentBmpFile);
      // playAudio(alignmentWavFile);

      if(nextButtonLoop()) {
        // User has confirmed alignment, move to zeroing
        prevState = currentState;
        currentState = ZEROING_PREP;
        currentGroupNum = 3;
      }

      // Now that one loop has passed, update prevState
      if(currentState == ALIGNMENT) {
        prevState = currentState;
        
      }

      break;
    
    case ZEROING_PREP:
      //currentGroupNum = 4;
      // Prepare for zeroing, then move to zeroing state
      // showScreen(zeroingPrepBmpFile);
      // playAudio(zeroingPrepWavFile);
      if (prevState != currentState) {
        calculateForceOffset();
        //DPRINTLN("Preparing for Zeroing...");
        //delay(1500); // Small delay to ensure any previous commands are finished
      }
  

      if(nextButtonLoop()) {
        // User has confirmed alignment, move to zeroing
        prevState = currentState;
        currentState = ZEROING;
        currentGroupNum = 4;
      }

      // Now that one loop has passed, update prevState
      if(currentState == ZEROING_PREP) {
        prevState = currentState;
        
      }

      break;
    
    case ZEROING:
      //currentGroupNum = 5;
      //showScreen(zeroingBmpFile);
      //playAudio(zeroingWavFile);
      // Initialize zeroing state, with error handling
      // Only do this when we switch states
      if(prevState != currentState) {
        DPRINTLN("Initializing Zeroing...");
        delay(1000); // Small delay to ensure any previous commands are finished
        initializeZeroing();
      }

      if(updateZeroing()) {
        // updateZeroing returned true, indicating zeroing is complete
        // Error cases handled in zeroing control
        DPRINTLN("ZEROING COMPLETE");
        prevState = currentState; //I changed this from COMPRESSIONS
        currentState = COMPRESSION_PREP;
        currentGroupNum = 5;
      }

            // Check for zeroing failure conditions
      if(linearPos > extensionStrokeLimit) {
          prevState = currentState;
          currentState = ABORT;
          currentGroupNum = 9; // Set to abort group
          //return false;
      }

      if(!readSensors()) {
        prevState = currentState;
        currentState = ABORT;
        currentGroupNum = 9;
        //return false;
    }
      
      // Now that one loop has passed, update prevState
      if(currentState == ZEROING) {
        prevState = currentState;
        
      }

      break;

    case COMPRESSION_PREP:
      //currentGroupNum = 6;
      // showScreen(compressionPrepBmpFile);
      // playAudio(compressionPrepWavFile);

      if(prevState != currentState) {
        prepTimer = millis();
      }

      if(millis() - prepTimer >= 1500) {
        prevState = currentState;
        currentState = COMPRESSIONS;
        currentGroupNum = 6;
      }
      // if (playWav1.isPlaying()) {
      //   audioWasPlaying = true;
      // }

      // // Trigger when playback *finishes*
      // if (audioWasPlaying && !playWav1.isPlaying()) {
      //     audioWasPlaying = false;  // reset
      //     prevState = currentState;
      //     currentState = COMPRESSIONS;
      // } 
      
      // Now that one loop has passed, update prevState
      if(currentState == COMPRESSION_PREP) {
        prevState = currentState;
     
      }

      break; 

    case COMPRESSIONS:
      //currentGroupNum = 7;
      // showScreen(compressionsBmpFile);
      // playAudio(compressionsWavFile);

      if(prevState != currentState) {
        //moteus.SetBrake();
        DPRINTLN("Initializing Compressions...");
        delay(500); // Small delay to ensure any previous commands are finished
        initializeCompressions();
      }

      // Error handling handled within updateCompressions()
      updateCompressions();

      // Check for pause command
      if(pauseButtonLoop()) {
        prevState = currentState;
        currentState = PAUSED;
        currentGroupNum = 7;
        moteus.SetBrake();
      }

      // Now that one loop has passed, update prevState
      if(currentState == COMPRESSIONS) {
        prevState = currentState;
      }

      break;

    case PAUSED:
      //currentGroupNum = 8;
      // showScreen(pausedBmpFile);
      // playAudio(pausedWavFile);
      // if(prevState != currentState) {
      //   moteus.SetBrake();
      // }

      retract();

      // If not paused, resume compressions
      if(nextButtonLoop()) {
        prevState = currentState;
        currentState = COMPRESSIONS;
        currentGroupNum = 6;
        moteus.SetStop();
      }

      // Now that one loop has passed, update prevState
      if(currentState == PAUSED) {
        prevState = currentState;
      }

      break;

    case KNEEL_FAILURE:
      //currentGroupNum = 9;
      // Command zero setpoint, don't necessarily need to get there before re-kneel
      //returnToCompressionZero(); 
      // showScreen(kneelFailureBmpFile);
      // playAudio(kneelFailureWavFile);

      if(prevState != currentState) {
        moteus.SetBrake();
      }
      // If they re-kneel, get back to rib breaking
      if(!isKneelingFailure()) {
        prevState = currentState;
        currentState = COMPRESSIONS;
        currentGroupNum = 6;
        moteus.SetBrake();
      }

      // Now that one loop has passed, update prevState
      if(currentState == KNEEL_FAILURE) {
        prevState = currentState;
      }

      break;

    case ABORT:
      //currentGroupNum = 9;
      // Shit's fucked, get plunger out of there and tell people to do manual compressions
      // showScreen(abortBmpFile);
      // playAudio(abortWavFile);
      //updateAbort();

      if(prevState != currentState) {
        DPRINTLN("ABORT");
        moteus.SetBrake();

        // Update prevState now, no backing out of this one
        prevState = currentState;
      }

      retract();
      
      // No getting out of this one, don't change state
      // TODO: Handle reset after use
      break;
  }

  // DPRINTLN(millis() - timer);
  // Serial.println(millis() - timer);
}
