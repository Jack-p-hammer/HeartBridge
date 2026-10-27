#include "Adafruit_RA8875.h"
#include <Adafruit_STMPE610.h>
#include <Adafruit_GFX.h>

void bmpDraw(Adafruit_RA8875 *tft, const char *filename, int x, int y, volatile bool *abortFlag = nullptr);
uint16_t read16(File f);
uint32_t read32(File f);
uint16_t color565(uint8_t r, uint8_t g, uint8_t b);
byte decToBcd(byte val);

// Pin definitions
extern const int SD_CHIP_SELECT;
extern const int BUTTON_PIN;
extern const int RA8875_CS;
extern const int RA8875_RESET;

// Display object
extern Adafruit_RA8875 tft;

// RA8875 SPI clock. The library defaults to 4 MHz on Teensy; 12 MHz is
// within the RA8875's limit for both register reads and writes.
#define RA8875_SPI_SPEED 12000000
extern uint32_t spi_speed; // defined in Adafruit_RA8875.cpp

// Button state variables
extern bool buttonState;
extern bool lastButtonReading;
extern unsigned long lastDebounceTime;
extern const unsigned long DEBOUNCE_DELAY;

