#pragma once
#include <Audio.h>

// Utility functions for CPR Machine operation
void HMI_util_setup();
bool verifyBatteryPercentage();
void displayStartUpInstructions();
void displayCutClothingInstructions();
void displayUnfoldInstructions();
bool checkUserConfirmation(); // True if user begins device operation
void displayAlignmentInstructions();
void displayZeroingInstructions();
bool displayCompressionConfirmation();
void showScreen(const char *file);
void playAudio(const char *wavFileName);
bool nextButtonLoop();
bool pauseButtonLoop();
float getBatteryPercentage();
void logBatteryData(); 

extern unsigned long GreenNow;
extern unsigned long PauseNow;

extern AudioPlaySdWav playWav1;

extern const char *startUpBmpFile;
extern const char *cutClothingBmpFile;
extern const char *unfoldBmpFile;
extern const char *alignmentBmpFile;
extern const char *zeroingPrepBmpFile;
extern const char *zeroingBmpFile;
extern const char *compressionPrepBmpFile;
extern const char *compressionsBmpFile;
extern const char *pausedBmpFile;
extern const char *kneelFailureBmpFile;
extern const char *abortBmpFile;


extern const char *startUpWavFile; 
extern const char *cutClothingWavFile;
extern const char *unfoldWavFile;
extern const char *alignmentWavFile;
extern const char *zeroingPrepWavFile;
extern const char *zeroingWavFile;
extern const char *compressionPrepWavFile;
extern const char *compressionsWavFile;
extern const char *pausedWavFile;
extern const char *kneelFailureWavFile;
extern const char *abortWavFile;


extern const char *frameGroups[11];
extern const char *wavGroups[11];
//extern int currentGroup;

extern bool audioBusy;
extern uint32_t audioStartedAt;

void showCurrentFrame(int currentGroup);
void playCurrentWav(int currentGroup);
extern void showCurrentFrameAndAudio(int currentGroup);