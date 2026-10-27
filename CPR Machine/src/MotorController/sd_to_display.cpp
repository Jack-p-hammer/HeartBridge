#include <SPI.h>
#include <Wire.h>
#include <SD.h>
#include "Adafruit_RA8875.h"
#include <Adafruit_STMPE610.h>
#include <Adafruit_GFX.h>
#include "sd_to_display.h"

#define MAX_ROW_PIXELS 800 // RA8875 800x480 panel width


void bmpDraw(Adafruit_RA8875 *tft, const char *filename, int x, int y, volatile bool *abortFlag) {
  File     bmpFile;
  int      bmpWidth, bmpHeight;   // W+H in pixels
  uint8_t  bmpDepth;              // Bit depth (currently must be 24)
  uint32_t bmpImageoffset;        // Start of image data in file
  uint32_t rowSize;               // Not always = bmpWidth; may have padding
  static uint8_t sdbuffer[3*MAX_ROW_PIXELS];  // one scanline in (B+G+R per pixel)
  static uint8_t lcdbuffer[2*MAX_ROW_PIXELS]; // one scanline out (RGB565, MSB first)
  boolean  goodBmp = false;       // Set to true on valid header parse
  boolean  flip    = true;        // BMP is stored bottom-to-top
  int      w, h, row, col;
  uint8_t  r, g, b;
  uint32_t pos = 0, startTime = millis();

  if((x >= tft->width()) || (y >= tft->height())) return;

  Serial.println();
  Serial.print(F("Loading image '"));
  Serial.print(filename);
  Serial.println('\'');

  // Open requested file on SD card
  if ((bmpFile = SD.open(filename)) == false) {
    Serial.println(F("File not found"));
    return;
  }

  // Parse BMP header
  if(read16(bmpFile) == 0x4D42) { // BMP signature
    Serial.println(F("File size: "));
    Serial.println(read32(bmpFile));
    (void)read32(bmpFile); // Read & ignore creator bytes
    bmpImageoffset = read32(bmpFile); // Start of image data
    Serial.print(F("Image Offset: "));
    Serial.println(bmpImageoffset, DEC);

    // Read DIB header
    Serial.print(F("Header size: "));
    Serial.println(read32(bmpFile));
    bmpWidth  = read32(bmpFile);
    bmpHeight = read32(bmpFile);

    if(read16(bmpFile) == 1) { // # planes -- must be '1'
      bmpDepth = read16(bmpFile); // bits per pixel
      Serial.print(F("Bit Depth: "));
      Serial.println(bmpDepth);
      if((bmpDepth == 24) && (read32(bmpFile) == 0)) { // 0 = uncompressed
        goodBmp = true; // Supported BMP format -- proceed!
        Serial.print(F("Image size: "));
        Serial.print(bmpWidth);
        Serial.print('x');
        Serial.println(bmpHeight);

        // BMP rows are padded (if needed) to 4-byte boundary
        rowSize = (bmpWidth * 3 + 3) & ~3;

        // If bmpHeight is negative, image is in top-down order.
        // This is not canon but has been observed in the wild.
        if(bmpHeight < 0) {
          bmpHeight = -bmpHeight;
          flip      = false;
        }

        // Crop area to be loaded
        w = bmpWidth;
        h = bmpHeight;
        if((x+w-1) >= tft->width())  w = tft->width()  - x;
        if((y+h-1) >= tft->height()) h = tft->height() - y;

        // Write pixels left-to-right. Set once here instead of on every
        // drawPixels() call, which also did a register read each time.
        tft->writeReg(RA8875_MWCR0,
          (tft->readReg(RA8875_MWCR0) & ~RA8875_MWCR0_DIRMASK) | RA8875_MWCR0_LRTD);

        for (row=0; row<h; row++) { // For each scanline...
          if(flip) // Bitmap is stored bottom-to-top order (normal BMP)
            pos = bmpImageoffset + (bmpHeight - 1 - row) * rowSize;
          else     // Bitmap is stored top-to-bottom
            pos = bmpImageoffset + row * rowSize;

          if (bmpFile.position() != pos) bmpFile.seek(pos);
          bmpFile.read(sdbuffer, w * 3); // Whole scanline in one read

          // Convert BGR888 to RGB565, high byte first as the RA8875 expects
          for (col=0; col<w; col++) {
            b = sdbuffer[col*3];
            g = sdbuffer[col*3 + 1];
            r = sdbuffer[col*3 + 2];
            uint16_t c = color565(r,g,b);
            lcdbuffer[col*2]     = c >> 8;
            lcdbuffer[col*2 + 1] = c & 0xFF;
          }

          // Send the whole scanline in one SPI burst
          tft->setXY(x, y + row);
          tft->writeCommand(RA8875_MRWC);
          SPI.beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE0));
          digitalWrite(RA8875_CS, LOW);
          SPI.transfer(RA8875_DATAWRITE);
          SPI.transfer(lcdbuffer, nullptr, w * 2);
          digitalWrite(RA8875_CS, HIGH);
          SPI.endTransaction();
        } // end scanline

        Serial.print(F("Loaded in "));
        Serial.print(millis() - startTime);
        Serial.println(" ms");

      } // end goodBmp
    }
  }

  bmpFile.close();
  if(!goodBmp) Serial.println(F("BMP format not recognized."));

}

// These read 16- and 32-bit types from the SD card file.
// BMP data is stored little-endian, Arduino is little-endian too.
// May need to reverse subscript order if porting elsewhere.

uint16_t read16(File f) {
  uint16_t result;
  ((uint8_t *)&result)[0] = f.read(); // LSB
  ((uint8_t *)&result)[1] = f.read(); // MSB
  return result;
}

uint32_t read32(File f) {
  uint32_t result;
  ((uint8_t *)&result)[0] = f.read(); // LSB
  ((uint8_t *)&result)[1] = f.read();
  ((uint8_t *)&result)[2] = f.read();
  ((uint8_t *)&result)[3] = f.read(); // MSB
  return result;
}

uint16_t color565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

byte decToBcd(byte val){
  // Convert normal decimal numbers to binary coded decimal
  return ( (val/10*16) + (val%10) );
}