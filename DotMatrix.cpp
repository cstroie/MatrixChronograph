/**
  DotMatrix.cpp - MAX7219/MAX7221 SPI interface

  Copyright (C) 2017-2018 Costin STROIE <costinstroie@eridu.eu.org>

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "Arduino.h"
#include <SPI.h>

#include "DotMatrix.h"

DotMatrix::DotMatrix() {
}

/**
  Initialize the matrices

  @param csPin the CipSelect pin
  @param devices the number of matrices
  @param lines the scan lines number
*/
void DotMatrix::init(uint8_t csPin, uint8_t devices, uint8_t lines) {
  /* Pin configuration */
  SPI_CS = csPin;
  /* The number of matrices */
  if (devices <= 0 || devices > MAX_MATRICES)
    devices = MAX_MATRICES;
  _devices = devices;
  /* Configure the pins and SPI */
  pinMode(MOSI, OUTPUT);
  pinMode(SCK, OUTPUT);
  pinMode(SPI_CS, OUTPUT);
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  SPI.begin();
  digitalWrite(SPI_CS, HIGH);
  /* Set the scan limit */
  this->scanlimit(lines);
  /* Set the maximum framebuffer size */
  maxFB = devices * lines;
}

/**
  Set the decoding mode

  @param value the decoding mode 0..0x0F
*/
void DotMatrix::decodemode(uint8_t value) {
  sendAllSPI(OP_DECODE, value & 0x0F);
}

/**
  Set the LED brightness

  @param value the brightness 0..0x0F
*/
void DotMatrix::intensity(uint8_t value) {
  if (value <= 0x0F)
    sendAllSPI(OP_INTENS, value);
}

/**
  Set the scan limit

  @param value the scan limit 0..0x07
*/
void DotMatrix::scanlimit(uint8_t value) {
  _scanlimit = ((value - 1) & 0x07) + 1;
  sendAllSPI(OP_SCNLMT, _scanlimit - 1);
}

/**
  LEDs off / on

  @param yesno the on/off switch
*/
void DotMatrix::shutdown(bool yesno) {
  uint8_t data = yesno ? 0 : 1;
  sendAllSPI(OP_SHTDWN, data);
}

/**
  Display test mode

  @param yesno the test on/off switch
*/
void DotMatrix::displaytest(bool yesno) {
  uint8_t data = yesno ? 1 : 0;
  sendAllSPI(OP_DSPTST, data);
}

/**
  Clear the matrix (all leds off)
*/
void DotMatrix::clear() {
  for (uint8_t l = 0; l < _scanlimit; l++)
    sendAllSPI(l + 1, 0x00);
}

/**
  Load the specified font into RAM and precompute character limits

  This function loads a font from PROGMEM into RAM and rotates each character
  for proper display orientation. It also precomputes the left/right limits 
  and width of each character for efficient rendering.

  @param font the font id (0-14)
*/
void DotMatrix::loadFont(uint8_t font) {
  uint8_t chrbuf[8] = {0};
  font %= fontCount;
  // Load each character into RAM
  for (int i = 0; i < fontChars; i++) {
    // Load into temporary buffer from PROGMEM
    memcpy_P(&chrbuf, &FONTS[font][i], 8);
    // Rotate each character 90 degrees clockwise for proper orientation
    // This converts from horizontal storage to vertical column format
    for (uint8_t j = 0; j < 8; j++) {
      for (uint8_t k = 0; k < 8; k++) {
        // Shift the target column left and add the LSB from source
        FONT[i][7 - k] <<= 1;
        FONT[i][7 - k] |= (chrbuf[j] & 0x01);
        // Shift the source right to process next bit
        chrbuf[j] >>= 1;
      }
    }
  }
  // Get the limits of the digits (use character '8' which has maximum width)
  chrLimits_t lmt = getLimits(0x08);
  // And use it for all the digits (0-9) for consistent spacing
  for (uint8_t c = 0; c <= 9; c++)
    chrLimits[c] = lmt;
  // Get the limits of all the other characters individually
  for (uint8_t c = 0x0A; c < fontChars; c++)
    chrLimits[c] = getLimits(c);
}

/**
  Get the character limits and width by analyzing non-zero columns

  This function determines the leftmost and rightmost non-zero columns
  in a character's bitmap to calculate the effective width and positioning.
  This allows for more compact character rendering by ignoring empty columns.

  @param ch the character to compute the limits and width for
  @return the character limits struct with right, left, and width values
*/
chrLimits_t DotMatrix::getLimits(uint8_t ch) {
  chrLimits_t lmt = {maxWidth, maxWidth, 0};
  // The characters are already rotated, just find the non-zero bytes
  for (uint8_t l = 0; l < maxWidth; l++) {
    // Check for non-zero column (contains lit LEDs)
    if (FONT[ch][l] != 0) {
      // Keep the rightmost limit (first non-zero column from left)
      if (lmt.right == maxWidth) lmt.right = l;
      // Keep the leftmost limit (last non-zero column)
      lmt.left = l;
    }
  }
  // If valid (character has visible pixels), compute the char width
  if (lmt.right != maxWidth)
    lmt.width = lmt.left - lmt.right + 1;
  // Return the result
  return lmt;
}

/**
  Clear the framebuffer
*/
void DotMatrix::fbClear() {
  memset(fbData, 0, maxFB);
}

/**
  Display the framebuffer
*/
void DotMatrix::fbDisplay() {
  /* Repeat for each line in matrix */
  for (uint8_t i = 0; i < _scanlimit; i++) {
    /* Compose an array containing the same line in all matrices */
    uint8_t data[_devices] = {0};
    /* Fill the array from the frambuffer */
    for (uint8_t m = 0; m < _devices; m++)
      data[m] = fbData[m * _scanlimit + i];
    /* Send the array */
    sendAllSPI(i + 1, data, _devices);
  }
}

/**
  Print a valid character at the specified position in the framebuffer

  This function renders a single character at a specific position in the
  framebuffer by OR-ing the character's bitmap with existing framebuffer data.
  It uses precomputed character limits to render only the visible portion
  of the character for efficiency.

  @param pos position (leftmost column) in framebuffer to start printing
  @param digit the character/digit to print (0-15)
*/
void DotMatrix::fbPrint(uint8_t pos, uint8_t digit) {
  // Print only if the character is valid
  if (digit < fontChars)
    // Process each column of the character using precomputed width
    for (uint8_t l = 0; l < chrLimits[digit].width; l++)
      // Print only if inside framebuffer bounds
      if (pos + l < maxFB)
        // OR the character column data with existing framebuffer data
        // l + chrLimits[digit].right adjusts for any leading empty columns
        fbData[pos + l] |= FONT[digit][l + chrLimits[digit].right] ;
}

/**
  Print the characters at the specified positions

  @param poss positions array
  @param digit the characters array to print
  @param len number of characters
  @param alogn print alignment
*/
void DotMatrix::fbPrint(uint8_t* poss, uint8_t* chars, uint8_t len) {
  // Clear the framebuffer
  fbClear();
  // Print each character at specified position on framebuffer
  for (uint8_t d = 0; d < len; d++)
    fbPrint(poss[d], chars[d]);
  // Display the framebuffer
  fbDisplay();
}

/**
  Print the characters with automatic positioning and alignment

  This function calculates optimal positions for a string of characters
  based on their individual widths and the specified alignment. It first
  computes right-aligned positions, then adjusts for center or left alignment
  as requested.

  @param chars the characters array to print
  @param len number of characters
  @param align print alignment (LEFT, CENTER, RIGHT)
*/
void DotMatrix::fbPrint(uint8_t* chars, uint8_t len, uint8_t align) {
  uint8_t poss[maxFB] = {0};
  uint8_t pos = 0;

  // First, compute the right-aligned positions by working backwards
  // This ensures proper spacing between characters
  for (int8_t d = len - 1; d >= 0; d--) {
    // Check if the character is valid and compute its print and next positions
    // using its limits or use the limits of the digits by default
    uint8_t chr = chars[d] < fontChars ? chars[d] : 0;
    poss[d] = pos;
    // Add character width plus one column spacing to position counter
    pos += chrLimits[chr].width + 1;
  }

  // Apply alignment adjustment if text fits in framebuffer
  if (align == CENTER and pos < maxFB) {
    // Get the offset to center the text
    uint8_t offset = (maxFB - (pos - 1)) / 2;
    // Add the offset to all positions
    for (uint8_t d = 0; d < len; d++)
      poss[d] += offset;
  }
  else if (align == LEFT and pos < maxFB) {
    // Get the offset to left-align the text
    uint8_t offset = maxFB - (pos - 1);
    // Add the offset to all positions
    for (uint8_t d = 0; d < len; d++)
      poss[d] += offset;
  }

  // Print each character at computed position on framebuffer
  fbPrint(poss, chars, len);
}

/**
  Send data to one device
*/
void DotMatrix::sendSPI(uint8_t matrix, uint8_t reg, uint8_t data) {
  uint8_t offset = matrix * 2;

  /* Clear the command buffer */
  memset(cmdBuffer, 0, _devices * 2);
  /* Write the data into command buffer */
  cmdBuffer[offset] = data;
  cmdBuffer[offset + 1] = reg;

  /* Chip select */
  digitalWrite(SPI_CS, LOW);
  /* Send the data */
  SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE0));
  for (int i = _devices * 2; i > 0; i--)
    SPI.transfer(cmdBuffer[i - 1]);
  SPI.endTransaction();
  /* Latch data */
  digitalWrite(SPI_CS, HIGH);
}

/**
  Send the same data to all device, at once
*/
void DotMatrix::sendAllSPI(uint8_t reg, uint8_t data) {
  /* Chip select */
  digitalWrite(SPI_CS, LOW);
  /* Send the data */
  SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE0));
  for (int i = _devices; i > 0; i--) {
    SPI.transfer(reg);
    SPI.transfer(data);
  }
  SPI.endTransaction();
  /* Latch data */
  digitalWrite(SPI_CS, HIGH);
}

/**
  Send the data in array to device, at once
*/
void DotMatrix::sendAllSPI(uint8_t reg, uint8_t* data, uint8_t size) {
  /* Chip select */
  digitalWrite(SPI_CS, LOW);
  /* Send the data */
  SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE0));
  for (int i = size; i > 0; i--) {
    SPI.transfer(reg);
    SPI.transfer(data[i - 1]);
  }
  SPI.endTransaction();
  /* Latch data */
  digitalWrite(SPI_CS, HIGH);
}

