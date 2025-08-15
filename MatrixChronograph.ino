/**
  LedMatrixClock - A desk clock using 4 8x8 led matrix modules

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

// EEPROM
#include <EEPROM.h>
// Watchdog, sleep
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <IRLremote.h>

#include "Button.h"
#include "DotMatrix.h"
#include "DS3231.h"

// Software name and vesion
const char DEVNAME[]  PROGMEM = "MatrixChronograph";
const char VERSION[]  PROGMEM = "v2.25";
const char AUTHOR[]   PROGMEM = "Costin Stroie <costinstroie@eridu.eu.org>";
const char DATE[]     PROGMEM = __DATE__;

// Pin definitions
const int CS_PIN    = 10; // ~SS
const int MOSI_PIN  = 11; // MOSI
const int MISO_PIN  = 12; // MISO
const int SCK_PIN   = 13; // SCK
const int BEEP_PIN  = 5;
const int BTN1_PIN  = 4;
const int BTN2_PIN  = 6;
const int INTSQ_PIN = A3;
const int IRED_PIN  = 3;
const int LIGHT_PIN = A0;

// Buttons
Button btn1(BTN1_PIN);
Button btn2(BTN2_PIN);
Button intsq(INTSQ_PIN);

// Choose the IR protocol of your remote
//CHashIR iRed;
CNec iRed;

// The RTC
DS3231 rtc;
uint32_t  mtxDisplayWait  = 1000UL;                             // Wait interval
uint32_t  mtxDisplayUntil = 0UL;                                // Display check until
bool      mtxDisplayNow   = true;                               // Force display

// The matrix object
#define MATRICES  4
#define SCANLIMIT 8
DotMatrix mtx = DotMatrix();

// Automatic brightness steps
uint32_t brgtCheckUntil = 0UL;
uint32_t brgtCheckWait  = 100UL;

// Display modes
enum      mtxModes {MODE_HHMM, MODE_SS, MODE_DDMM, MODE_YY,
                    MODE_TEMP, MODE_VCC, MODE_MCU, MODE_SET_TIME, MODE_SET_DATE, MODE_ALL
                   };
uint8_t   mtxMode       = MODE_HHMM;                            // Initial mode
uint32_t  mtxModeUntil  = 0UL;                                  // Expiration time, 0 is never
uint32_t  mtxModeWait   = 10000UL;                              // Expiration interval

// Time setting variables
uint8_t   setHours      = 0;
uint8_t   setMinutes    = 0;
uint8_t   setDay        = 0;
uint8_t   setMonth      = 0;
uint16_t  setYear       = 0;
uint8_t   setTimeField  = 0;                                    // 0=hours, 1=minutes
uint8_t   setDateField  = 0;                                    // 0=day, 1=month, 2=year

// Define the configuration type
struct cfgEE_t {
  union {
    struct {
      uint8_t font: 4;  // Display font
      uint8_t brgt: 4;  // Display brightness (manual)
      uint8_t mnbr: 4;  // Minimal display brightness (auto)
      uint8_t mxbr: 4;  // Minimum display brightness (auto)
      uint8_t aubr: 1;  // Manual/Auto display brightness adjustment
      uint8_t tmpu: 1;  // Temperature units
      uint8_t spkm: 2;  // Speaker mode
      uint8_t spkl: 2;  // Speaker volume level
      uint8_t echo: 1;  // Local echo
      uint8_t dst:  1;  // DST flag
      int8_t  kvcc: 8;  // Bandgap correction factor (/1000)
      int8_t  ktmp: 8;  // MCU temperature correction factor (/100)
      uint8_t scqt: 1;  // Serial console quiet mode (negate)
      uint8_t bfst: 5;  // First hour to beep
      uint8_t blst: 5;  // Last hour to beep
    };
    uint8_t data[8];    // We use 8 bytes in the structure
  };
  uint8_t crc8;         // CRC8
};
// The factory default configuration
const cfgEE_t cfgDefault = {{{
      .font = 0x01, .brgt = 0x01, .mnbr = 0x00, .mxbr = 0x0F,
      .aubr = 0x01, .tmpu = 0x01, .spkm = 0x01, .spkl = 0x01,
      .echo = 0x01, .dst  = 0x00, .kvcc = 0x00, .ktmp = 0x00,
      .scqt = 0x00, .bfst = 0x08, .blst = 0x14,
    }
  }
};
// The global configuration structure
struct cfgEE_t  cfgData;
// EEPROM address to store the configuration to
uint16_t        cfgEEAddress = 0x0180;

// Several Hayes related globals
const int8_t  HAYES_NUM_ERROR = -128;
// String buffer
char buf[65] = "";
// Line buffer length
int8_t len = -1;
// Buffer index
uint8_t idx = 0;
// Numeric value
int8_t value = 0;
// Command result
bool result = false;


/**
  Print a character array from program memory

  @param str the character array to print
  @param eol print the EOL
*/
void print_P(const char *str, bool eol = false) {
  uint8_t val;
  do {
    val = pgm_read_byte(str++);
    if (val) Serial.write(val);
  } while (val);
  if (eol) Serial.println();
}

/**
  Parse the buffer and return an integer

  @param buf the char buffer to parse
  @param idx index to start with
  @param len maximum to parse, 0 for no limit
  @return the integer found or zero
*/
int16_t getInteger(char* buf, int8_t idx, uint8_t len = 32) {
  int16_t result = 0;     // The result
  bool isNeg = false;     // Negative flag
  static uint8_t ldx = 0; // Local index, static
  uint8_t sdx = 0;        // Start index

  // If the specified index is negative, use the last local index;
  // if it is positive, re-set the local index
  if (idx >= 0) {
    // Use the new index
    ldx = idx;
    sdx = idx;
  }
  // Preamble: find the first sign or digit character
  while ((ldx - sdx < len)
         and (buf[ldx] != 0)
         and (buf[ldx] != '-')
         and (buf[ldx] != '+')
         and (not isdigit(buf[ldx])))
    ldx++;
  // At this point, check if we have ended the buffer or the length
  if ((ldx - sdx < len) and (buf[ldx] != 0)) {
    // Might start with '+' or '-', keep the sign and move forward
    // only if the sign is the first character ever
    if (buf[ldx] == '-') {
      if (idx >= 0) isNeg = true;
      ldx++;
    }
    else if (buf[ldx] == '+') {
      isNeg = false;
      ldx++;
    }
    // Parse the digits
    while ((isdigit(buf[ldx]))
           and (ldx - sdx < len)
           and (buf[ldx] != 0)) {
      // Build the result
      result = (result * 10) + (buf[ldx] - '0');
      // Move forward
      ldx++;
    }
    // Check if negative
    if (isNeg)
      result = -result;
  }
  // Return the result
  return result;
}

/**
  Parse the buffer and return the integer value if it fits in the specified interval
  or the default value if not.

  @param buf the char buffer to parse
  @param idx index to start with
  @param low minimal valid value
  @param hgh maximal valid value
  @param def default value
  @param len maximum to parse, 0 for no limit
  @return the integer value or default
*/
int16_t getValidInteger(char* buf, int8_t idx, int16_t low, int16_t hgh, int16_t def = 0, uint8_t len = 32) {
  // Get the integer value
  int16_t res = getInteger(buf, idx, len);
  // Check if valid
  if ((res < low) or (res > hgh))
    res = def;
  // Return the result
  return res;
}

/**
  Parse the buffer and return one digit integer

  @param buf the char buffer to parse
  @param idx index to start with
  @return the integer found or HAYES_NUM_ERROR
*/
int8_t getDigit(char* buf, int8_t idx) {
  // The result is signed integer
  int8_t res = HAYES_NUM_ERROR;
  // Make sure the pointed char is a digit and get the numeric value
  if (isdigit(buf[idx]))
    res = buf[idx] - '0';
  // Return the result
  return res;
}

/**
  Parse the buffer and return the digit value if it fits in the specified interval
  or the default value if not.

  @param buf the char buffer to parse
  @param idx index to start with
  @param low minimal valid value
  @param hgh maximal valid value
  @param def default value
  @return the digit value or default
*/
int8_t getValidDigit(char* buf, int8_t idx, int8_t low, int8_t hgh, int8_t def = HAYES_NUM_ERROR) {
  // Get the digit value
  int8_t res = getDigit(buf, idx);;
  // Check if valid
  if ((res != HAYES_NUM_ERROR) and
      ((res < low) or (res > hgh)))
    res = def;
  // Return the result
  return res;
}

/**
  CRC8 computing

  @param inCrc partial CRC
  @param inData new data
  @return updated CRC
*/
uint8_t CRC8(uint8_t inCrc, uint8_t inData) {
  uint8_t outCrc = inCrc ^ inData;
  for (uint8_t i = 0; i < 8; ++i) {
    if ((outCrc & 0x80 ) != 0) {
      outCrc <<= 1;
      outCrc ^= 0x07;
    }
    else {
      outCrc <<= 1;
    }
  }
  return outCrc;
}

/**
  Compute the CRC8 of the configuration structure

  @param cfgTemp the configuration structure
  @return computed CRC8
*/
uint8_t cfgEECRC(struct cfgEE_t cfg) {
  // Compute the CRC8 checksum of the data
  uint8_t crc8 = 0;
  for (uint8_t i = 0; i < sizeof(cfg) - 1; i++)
    crc8 = CRC8(crc8, cfg.data[i]);
  return crc8;
}

/**
  Compare two configuration structures

  @param cfg1 the first configuration structure
  @param cfg1 the second configuration structure
  @return true if equal
*/
bool cfgEqual(struct cfgEE_t cfg1, struct cfgEE_t cfg2) {
  bool result = true;
  // Check CRC first
  if (cfg1.crc8 != cfg2.crc8)
    result = false;
  else
    // Compare the overlayed array of the two structures
    for (uint8_t i = 0; i < sizeof(cfg1) - 1; i++)
      if (cfg1.data[i] != cfg2.data[i])
        result = false;
  return result;
}

/**
  Write the configuration to EEPROM, along with CRC8, if different
*/
bool cfgWriteEE() {
  // Temporary configuration structure
  struct cfgEE_t cfgTemp;
  // Read the data from EEPROM into the temporary structure
  EEPROM.get(cfgEEAddress, cfgTemp);
  // Compute the CRC8 checksum of the read data
  uint8_t crc8 = cfgEECRC(cfgTemp);
  // Compute the CRC8 checksum of the actual data
  cfgData.crc8 = cfgEECRC(cfgData);
  // Compare the new and the stored data and check if the stored data is valid
  if (not cfgEqual(cfgData, cfgTemp) or (cfgTemp.crc8 != crc8))
    // Write the data
    EEPROM.put(cfgEEAddress, cfgData);
  // Always return true, even if data is not written
  return true;
}

/**
  Read the configuration from EEPROM, along with CRC8, and verify
*/
bool cfgReadEE(bool useDefaults = false) {
  // Temporary configuration structure
  struct cfgEE_t cfgTemp;
  // Read the data from EEPROM into the temporary structure
  EEPROM.get(cfgEEAddress, cfgTemp);
  // Compute the CRC8 checksum of the read data
  uint8_t crc8 = cfgEECRC(cfgTemp);
  // And compare with the read crc8 checksum
  if      (cfgTemp.crc8 == crc8)  cfgData = cfgTemp;
  else if (useDefaults)           cfgDefaults();
  return (cfgTemp.crc8 == crc8);
}

/**
  Reset the configuration to factory defaults
*/
bool cfgDefaults() {
  cfgData = cfgDefault;
};


/**
  Analog raw reading, after a delay, while sleeping, using interrupt

  @return raw analog read value (long)
*/
long readRaw() {
  // Wait for voltage to settle (bandgap stabilizes in 40-80 us)
  delay(10);
  // Start conversion
  ADCSRA |= _BV(ADSC);
  // Wait to finish
  while (bit_is_set(ADCSRA, ADSC));
  // Reading register "ADCW" takes care of how to read ADCL and ADCH
  long wADC = ADCW;
  // The returned reading
  return wADC;
}

/**
  Read the analog pin after a delay, while sleeping, using interrupt

  @param pin the analog pin
  @return raw analog read value
*/
int readAnalog(uint8_t pin) {
  // Allow for channel or pin numbers
  if (pin >= 14) pin -= 14;

  // Set the analog reference to DEFAULT, select the channel (low 4 bits).
  // This also sets ADLAR (left-adjust result) to 0 (the default).
  ADMUX = _BV(REFS0) | (pin & 0x07);

  // Raw analog read
  long wADC = readRaw();

  // The returned reading
  return (int)(wADC);
}

/**
  Read the internal MCU temperature
  The internal temperature has to be used with the internal reference of 1.1V.
  Channel 8 can not be selected with the analogRead function yet.

  @return temperature in hundredths of degrees Celsius, *calibrated for my device*
*/
int16_t readMCUTemp(int8_t k = 0) {
  // Set the internal reference and mux.
  ADMUX = (_BV(REFS1) | _BV(REFS0) | _BV(MUX3));

  // Raw analog read
  long wADC = readRaw();

  // The returned temperature is in hundredths degrees Celsius; not calibrated
  return (int16_t)(100 * wADC - 27315L + k * 100);
}

/*
  Read the power supply voltage, by measuring the internal 1V1 reference

  @param k relative bandgap adjustment (/1000)
  @return voltage in millivolts, uncalibrated
*/
uint16_t readVcc(int8_t k = 0) {
  // Set the reference to Vcc and the measurement to the internal 1.1V reference
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);

  // Raw analog read
  long wADC = readRaw();

  // Return Vcc in mV; 1125300 = 1.1 * 1024 * 1000
  return (uint16_t)(1.1 * 1024.0 * (1000 + k) / wADC);
}

/**
  Compute the brightness
*/
uint8_t brightness() {
  // Use a static variable, one-time initialized with currently
  // read analog value from LDR
  static uint16_t lstLight = readAnalog(LIGHT_PIN);
  // Use another static variable, one-time initialized with currently
  // computed brightness value
  static uint8_t lstBrght = map(lstLight, 0, 1023, cfgData.mxbr, cfgData.mnbr);
  // Check for auto or manual adjustments
  if (cfgData.aubr) {
    // Automatic, read the LDR connected to LIGHT_PIN
    uint16_t nowLight = readAnalog(LIGHT_PIN);
    // Exponential smooth 3.125%
    lstLight = ((lstLight << 4) - lstLight + nowLight + 8) >> 4;
    // Map in min..max range
    uint8_t brght = map(lstLight, 0, 1023, cfgData.mxbr, cfgData.mnbr);
    // If not changed, return an invalid value
    if (brght == lstBrght)
      return 0xFF;
    else {
      // Else, keep the current value and return
      Serial.print(F("*B: ")); Serial.println(brght);
      lstBrght = brght;
      return brght;
    }
  }
  else
    // Manual
    return cfgData.brgt;
}

/**
  Simple short BEEP

  @param duration beep duration in ms
*/
void beep(uint16_t duration = 10) {
  if (cfgData.spkl > 0) {
    pinMode(BEEP_PIN, OUTPUT);
    digitalWrite(BEEP_PIN, HIGH);
    delay(duration * sq(cfgData.spkl));
    digitalWrite(BEEP_PIN, LOW);
  }
}

/**
  Show the time specfied in unpacked BCD (4 bytes)

  @param HHMM 4-bytes array containing hours and minutes in unpacked BCD format
*/
void showTimeBCD(uint8_t* HHMM) {
  // Create a new array, containing the hours (2 digits), the colon symbol and
  // the minutes (2 digits)
  uint8_t data[] = {HHMM[0], HHMM[1], 0x0A, HHMM[2], HHMM[3]};
  // Print on framebuffer
  mtx.fbPrint(data, sizeof(data) / sizeof(*data));
  // Print to console
  if (not cfgData.scqt) {
    Serial.print(F("*O")); Serial.print(MODE_HHMM); Serial.print(F(": "));
    Serial.print(data[0], 10); Serial.print(data[1], 10); Serial.print(F(":"));
    Serial.print(data[3], 10); Serial.print(data[4], 10); Serial.println();
  }
}

/**
  Show the time

  @param hh hours   0..23
  @param mm minutes 0..59
*/
void showTime(uint8_t hh, uint8_t mm) {
  // Convert hhmm to unpacked BCD, 4 digits
  uint8_t HHMM[4] = {hh / 10, hh % 10, mm / 10, mm % 10};

  // Display the time in unpacked BCD
  showTimeBCD(HHMM);
}

/**
  Check and display mode HHMM
*/
void showModeHHMM() {
  if (rtc.rtcOk) {
    // Check the alarms, the Alarm 2 triggers once per minute
    if ((rtc.checkAlarms() & 0x02) or mtxDisplayNow) {
      // Read the RTC, in BCD format
      bool newHour = rtc.readTimeBCD();
      // Check DST adjustments each new hour and read RTC if adjusted
      if (newHour)
        if (checkDST())
          rtc.readTimeBCD();
      // Show the time
      showTimeBCD(rtc.R);
      // Beep each hour, on the dot
      if (newHour) {
        // Get the hour in binary
        uint8_t hh = 10 * rtc.R[0] + rtc.R[1];
        // Check the Speaker mode switch and hours interval
        if (((cfgData.spkm == 1) and (hh >= cfgData.bfst) and (hh <= cfgData.blst)) or
            (cfgData.spkm == 2))
          beep();
      }
    }
  }
}

/**
  Display mode SS
*/
void showModeSS() {
  // Read the seconds from RTC, as BCD
  if (rtc.rtcOk) {
    // Read seconds
    uint8_t bcdSS = rtc.readSecondsBCD();
    // Convert to unpacked BCD, colon and 2 digits
    uint8_t data[] = {0xFF, 0xFF, 0x0A, bcdSS / 0x10, bcdSS % 0x10};
    // Print on framebuffer
    mtx.fbPrint(data, sizeof(data) / sizeof(*data));
    // Print to console
    if (not cfgData.scqt) {
      Serial.print(F("*O")); Serial.print(MODE_SS); Serial.print(F(": "));
      Serial.print(data[3], 10); Serial.print(data[4], 10); Serial.println();
    }
  }
}

/**
  Display mode DDMM
*/
void showModeDDMM() {
  // Read the full RTC time and date
  if (rtc.rtcOk and rtc.readTime(true)) {
    // Convert to unpacked BCD, day (2 digits), dot, month (2 digits)
    uint8_t data[] = {rtc.d / 10, rtc.d % 10, 0x0B, rtc.m / 10, rtc.m % 10};
    // Print on framebuffer
    mtx.fbPrint(data, sizeof(data) / sizeof(*data));
    // Print to console
    if (not cfgData.scqt) {
      Serial.print(F("*O")); Serial.print(MODE_DDMM); Serial.print(F(": "));
      Serial.print(data[0], 10); Serial.print(data[1], 10); Serial.print(F("."));
      Serial.print(data[3], 10); Serial.print(data[4], 10); Serial.println();
    }
  }
}

/**
  Check and display mode YY
*/
void showModeYY() {
  // Read the full RTC time and date
  if (rtc.rtcOk and rtc.readTime(true)) {
    // Convert to unpacked BCD, 4 digits
    uint8_t data[] = {rtc.Y / 1000, (rtc.Y % 1000) / 100, (rtc.Y % 100) / 10, rtc.Y % 10};
    // Print on framebuffer
    mtx.fbPrint(data, sizeof(data) / sizeof(*data));
    // Print to console
    if (not cfgData.scqt) {
      Serial.print(F("*O")); Serial.print(MODE_YY); Serial.print(F(": "));
      Serial.println(rtc.Y);
    }
  }
}

/**
  Display the mode TEMP
*/
void showModeTEMP() {
  // Get the temperature
  int8_t temp = rtc.readTemperature(cfgData.tmpu);
  // Absolute value
  uint8_t atemp = abs(temp);
  if (atemp >= 100) {
    // Create an array with the sign, value (3 digits) the degree symbol and units letter
    uint8_t data[] = {temp < 0 ? 0x0E : 0xFF, atemp / 100, (atemp % 100) / 10, atemp % 10, 0x0D, cfgData.tmpu ? 0x0C : 0x0F};
    // Print on framebuffer
    mtx.fbPrint(data, sizeof(data) / sizeof(*data));
  }
  else {
    // Create an array with the sign, value (2 digits) the degree symbol and units letter
    uint8_t data[] = {temp < 0 ? 0x0E : 0xFF, atemp / 10,  atemp % 10, 0x0D, cfgData.tmpu ? 0x0C : 0x0F};
    // Print on framebuffer
    mtx.fbPrint(data, sizeof(data) / sizeof(*data));
  }
  // Print to console
  if (not cfgData.scqt) {
    Serial.print(F("*O")); Serial.print(MODE_TEMP); Serial.print(F(": "));
    Serial.print(temp); Serial.println(cfgData.tmpu ? "C" : "F");
  }
}
/**
  Check and display mode VCC (supply voltage)
*/
void showModeVCC() {
  // Get the Vcc, mV
  int16_t vcc = readVcc(cfgData.kvcc);
  // Create an array with the value in Volts, with decimal dot
  uint8_t data[] = {vcc / 1000, 0x0B, (vcc % 1000) / 100, (vcc % 100) / 10, vcc % 10};
  // Print on framebuffer
  mtx.fbPrint(data, sizeof(data) / sizeof(*data));
  // Print to console
  if (not cfgData.scqt) {
    Serial.print(F("*O")); Serial.print(MODE_VCC); Serial.print(F(": "));
    Serial.print(vcc); Serial.println(F("mV"));
  }
}

/**
  Check and display mode MCU (MCU temperature)
*/
void showModeMCU() {
  // Get the MCU temperature
  int16_t temp = readMCUTemp(cfgData.ktmp);
  // Convert to Fahrenheit, if required
  if (not cfgData.tmpu)
    temp = (int16_t)((float)temp / 100 * 1.8 + 32.0);
  else
    // Use integer Celsius degrees
    temp /= 100;
  // Absolute value
  uint16_t atemp = abs(temp);
  if (atemp >= 100) {
    // Create an array with the sign, value (3 digits) the degree symbol and units letter
    uint8_t data[] = {temp < 0 ? 0x0E : 0xFF, atemp / 100, (atemp % 100) / 10, atemp % 10, 0x0D, cfgData.tmpu ? 0x0C : 0x0F};
    // Print on framebuffer
    mtx.fbPrint(data, sizeof(data) / sizeof(*data));
  }
  else {
    // Create an array with the sign, value (2 digits) the degree symbol and units letter
    uint8_t data[] = {temp < 0 ? 0x0E : 0xFF, atemp / 10,  atemp % 10, 0x0D, cfgData.tmpu ? 0x0C : 0x0F};
    // Print on framebuffer
    mtx.fbPrint(data, sizeof(data) / sizeof(*data));
  }
  // Print to console
  if (not cfgData.scqt) {
    Serial.print(F("*O")); Serial.print(MODE_MCU); Serial.print(F(": "));
    Serial.print(temp); Serial.println(cfgData.tmpu ? "C" : "F");
  }
}

/**
  Display Version
*/
void showModeVers() {
  // Local buffer
  uint8_t data[16] = "";
  // Get the data from PROGMEM
  strncpy_P(data, VERSION, 16);
  // Convert the characters
  uint8_t i;
  for (i = 0; i < 16; i++) {
    if (data[i] == 0)
      break;
    else if (data[i] == '.')
      data[i] = 0x0B;
    else if (isdigit(data[i]))
      data[i] -= '0';
    else
      data[i] = 0xFF;
  }
  // Print on framebuffer, right-aligned
  mtx.fbPrint(data, i, mtx.RIGHT);
}

/**
  Display time setting mode with visual field indication

  This function shows the current time setting values on the display with
  the active field (hours or minutes) visually indicated. In a future
  implementation, the active field could blink or be highlighted.

  @param hh hours value to display (0-23)
  @param mm minutes value to display (0-59)
*/
void showSetTime(uint8_t hh, uint8_t mm) {
  // Convert hhmm to unpacked BCD, 4 digits for display
  uint8_t HHMM[4] = {hh / 10, hh % 10, mm / 10, mm % 10};
  // Create a new array, containing the hours (2 digits), the colon symbol and
  // the minutes (2 digits)
  uint8_t data[] = {HHMM[0], HHMM[1], 0x0A, HHMM[2], HHMM[3]};
  
  // Add indicator for which field is being edited
  if (setTimeField == 0) {
    // Hours field is active (could blink in future implementation)
  } else {
    // Minutes field is active (could blink in future implementation)
  }
  
  // Print on framebuffer
  mtx.fbPrint(data, sizeof(data) / sizeof(*data));
}

/**
  Display date setting mode with field-specific formatting

  This function displays either the day/month or year depending on which
  field is currently being edited. The setDateField variable determines:
  - 0: Editing day
  - 1: Editing month  
  - 2: Editing year

  @param day day value to display (1-31)
  @param month month value to display (1-12)
  @param year year value to display (2000-2099)
*/
void showSetDate(uint8_t day, uint8_t month, uint16_t year) {
  if (setDateField == 2) {
    // Show year as 4-digit number
    uint8_t data[] = {year / 1000, (year % 1000) / 100, (year % 100) / 10, year % 10};
    mtx.fbPrint(data, sizeof(data) / sizeof(*data));
  } else {
    // Show day and month in DD.MM format
    uint8_t data[] = {day / 10, day % 10, 0x0B, month / 10, month % 10};
    mtx.fbPrint(data, sizeof(data) / sizeof(*data));
  }
}

/**
  Set the display mode

  @param mode the chosen mode
*/
void mtxSetMode(uint8_t mode) {
  if (mode == 0xFF) mode = MODE_ALL - 1;
  mtxMode = mode % MODE_ALL;
  if (mtxMode <= MODE_SS) mtxModeUntil =  0UL;                    // Never expire for HHMM and SS
  else                    mtxModeUntil =  millis() + mtxModeWait; // Expire after a while
  // Force display
  mtxDisplayNow = true;
}

/**
  Set the display to next mode
*/
void mtxNextMode() {
  mtxSetMode(mtxMode + 1);
}

/**
  Set the display to previous mode
*/
void mtxPrevMode() {
  mtxSetMode(mtxMode - 1);
}

/**
  AT-Hayes style command processing
*/
void handleHayes() {
  char c;
  // Read from serial only if there is room in buffer
  if (len < 64) {
    c = Serial.read();
    // Check if we have a valid character
    if (c >= 0) {
      // Uppercase
      c = toupper(c);
      // Local terminal command mode echo
      if (cfgData.echo)
        Serial.write(c);
      // Check the character
      if (c == '\b' and len > 0)
        // Backspace
        len--;
      else
        // Append to buffer
        buf[len++] = c;
      // Check for EOL
      if (c == '\r' or c == '\n') {
        // Flush the rest
        Serial.flush();
        // Send the newline
        Serial.println(F("\r\n"));
        // Make sure the last char is null
        buf[--len] = '\0';
        // Check the line length before processing
        if (len > 0)
          // Parse the line
          doCommand();
        // Reset the buffer length
        len = 0;
      }
    }
  }
}

void doCommand() {
  // Start by finding the 'AT' sequence
  char *pch = strstr_P(buf, PSTR("AT"));
  if (pch == NULL)
    return;

  // Jump over those two chars from the start
  idx = pch - buf + 2;
  // New line, just "AT"
  if (len <= 2)
    result = true;

  // Check the first character, could be a symbol or a letter
  switch (buf[idx++]) { // idx++ -> 1
    // Our extension, one letter and 1..2 digits (0-99), or '?', '='
    case '*':
      switch (buf[idx++]) { // idx++ -> 2
        // Auto brightness switch
        case 'A':
          if (buf[idx] == '?') {
            // Get brightness
            Serial.print(F("*A: ")); Serial.println(cfgData.aubr);
            result = true;
          }
          else if (len == idx) {
            cfgData.aubr = 0x00;
            mtx.intensity(brightness());
            result = true;
          }
          else {
            // Get the digit value
            value = getValidDigit(buf, idx, 0, 1, HAYES_NUM_ERROR);
            if (value != HAYES_NUM_ERROR) {
              // Set auto brightness
              cfgData.aubr = value;
              mtx.intensity(brightness());
              result = true;
            }
          }
          break;

        // Brightness
        case 'B':
          if (buf[idx] == '?') {
            // Get brightness
            Serial.print(F("*B: ")); Serial.println(cfgData.brgt);
            result = true;
          }
          else {
            // Get the integer value
            value = getValidInteger(buf, idx, 0, 15, HAYES_NUM_ERROR);
            if (value != HAYES_NUM_ERROR) {
              cfgData.aubr = false;
              cfgData.brgt = value;
              // Set the brightness
              mtx.intensity(brightness());
              result = true;
            }
          }
          break;

        // DST switch
        case 'D':
          if (buf[idx] == '?') {
            // Get DST
            Serial.print(F("*D: ")); Serial.println(cfgData.dst);
            result = true;
          }
          else if (len == idx) {
            cfgData.dst = 0x00;
            mtxDisplayNow = true;
            result = true;
          }
          else {
            // Get the digit value
            value = getValidDigit(buf, idx, 0, 1, HAYES_NUM_ERROR);
            if (value != HAYES_NUM_ERROR) {
              // Set DST
              cfgData.dst = value;
              mtxDisplayNow = true;
              result = true;
            }
          }
          break;

        // Latest hour to beep
        case 'E':
          if (buf[idx] == '?') {
            // Get latest hour
            Serial.print(F("*E: ")); Serial.println(cfgData.blst);
            result = true;
          }
          else {
            // Get the integer value
            value = getValidInteger(buf, idx, 0, 23, HAYES_NUM_ERROR);
            if (value != HAYES_NUM_ERROR) {
              // Set the minimum brightness
              cfgData.blst = value;
              result = true;
            }
          }
          break;

        // Font
        case 'F':
          if (buf[idx] == '?') {
            // Get font
            Serial.print(F("*F: ")); Serial.println(cfgData.font);
            result = true;
          }
          else {
            // Get the integer value
            value = getValidInteger(buf, idx, 0, fontCount - 1, HAYES_NUM_ERROR);
            if (value != HAYES_NUM_ERROR) {
              cfgData.font = value;
              // Set the font
              mtx.loadFont(cfgData.font);
              result = true;
            }
          }
          break;

        // Highest (maximum) auto brightness level
        case 'H':
          if (buf[idx] == '?') {
            // Get higher limit
            Serial.print(F("*H: ")); Serial.println(cfgData.mxbr);
            result = true;
          }
          else if (len == idx) {
            cfgData.mxbr = 0x0F;
            mtx.intensity(brightness());
            result = true;
          }
          else {
            // Get the integer value
            value = getValidInteger(buf, idx, 0, 15, HAYES_NUM_ERROR);
            if (value != HAYES_NUM_ERROR) {
              // Set the maximum brightness
              cfgData.mxbr = value;
              mtx.intensity(brightness());
              result = true;
            }
          }
          break;

        // Lowest (minimum) auto brightness level
        case 'L':
          if (buf[idx] == '?') {
            // Get lower limit
            Serial.print(F("*L: ")); Serial.println(cfgData.mnbr);
            result = true;
          }
          else if (len == idx) {
            cfgData.mnbr = 0x00;
            mtx.intensity(brightness());
            result = true;
          }
          else {
            // Get the integer value
            value = getValidInteger(buf, idx, 0, 15, HAYES_NUM_ERROR);
            if (value != HAYES_NUM_ERROR) {
              // Set the minimum brightness
              cfgData.mnbr = value;
              mtx.intensity(brightness());
              result = true;
            }
          }
          break;

        // MCU temperature correction factor
        case 'M':
          if (len == idx or buf[idx] == '?') {
            Serial.print(F("*M: ")); Serial.println(cfgData.ktmp);
            result = true;
          }
          else {
            // Get the integer value
            value = getValidInteger(buf, idx, -127, 127, HAYES_NUM_ERROR);
            if (value != HAYES_NUM_ERROR) {
              // Set the temperature correction factor
              cfgData.ktmp = value;
              result = true;
            }
          }
          break;

        // Display mode selection
        case 'O':
          if (buf[idx] == '?') {
            // Get mode
            Serial.print(F("*O: ")); Serial.println(mtxMode);
            result = true;
          }
          else if (len == idx) {
            mtxSetMode(MODE_HHMM);
            result = true;
          }
          else {
            // Get the integer value
            value = getValidInteger(buf, idx, 0, MODE_ALL - 1, HAYES_NUM_ERROR);
            if (value != HAYES_NUM_ERROR) {
              // Set the mode
              mtxSetMode(value);
              result = true;
            }
          }
          break;

        // First hour to beep
        case 'S':
          if (buf[idx] == '?') {
            // Get first hour
            Serial.print(F("*S: ")); Serial.println(cfgData.bfst);
            result = true;
          }
          else {
            // Get the integer value
            value = getValidInteger(buf, idx, 0, 23, HAYES_NUM_ERROR);
            if (value != HAYES_NUM_ERROR) {
              // Set the minimum brightness
              cfgData.bfst = value;
              result = true;
            }
          }
          break;

        // Time and date setting and query
        case 'T':
          //  Usage: AT*T="YYYY/MM/DD HH:MM:SS" or, using date(1),
          //  ( sleep 2 && date "+AT\*T=\"%Y/%m/%d %H:%M:%S\"" ) > /dev/ttyUSB0
          if (buf[idx] == '?') {
            // Get time and date
            rtc.readTime(true);
            Serial.print(rtc.Y); Serial.print(F("/"));
            Serial.print(rtc.m); Serial.print(F("/"));
            Serial.print(rtc.d); Serial.print(F(" "));
            Serial.print(rtc.H); Serial.print(F(":"));
            Serial.print(rtc.M); Serial.print(F(":"));
            Serial.print(rtc.S); Serial.println();
            result = true;
          }
          else if (buf[idx] == '=') {
            // Set time and date
            uint16_t  year    = getInteger(buf, idx);
            uint8_t   month   = getValidInteger(buf, -1, 1, 12, 0);
            uint8_t   day     = getValidInteger(buf, -1, 1, 31, 0);
            uint8_t   hour    = getValidInteger(buf, -1, 0, 24, 0);
            uint8_t   minute  = getValidInteger(buf, -1, 0, 60, 0);
            uint8_t   second  = getValidInteger(buf, -1, 0, 60, 0);
            if (year >= 1900  and year < 2100   and
                month >= 1    and month <= 12   and
                day >= 1      and day <= 31     and
                hour >= 0     and hour <= 23    and
                minute >= 0   and minute <= 59  and
                second >= 0   and second <= 59 ) {
              // The date is quite valid, set the clock to 00:00:00 if not
              // specified
              rtc.writeDateTime(second, minute, hour, day, month, year);
              // Check if DST and set the flag
              cfgData.dst = rtc.dstCheck(year, month, day, hour);
              // Store the configuration
              cfgWriteEE();
              result = true;
            }
          }
          break;

        // Temperature units and query
        case 'U':
          if (len == idx or buf[idx] == '?') {
            // Get temperature units
            Serial.print(F("*U: ")); Serial.println(cfgData.tmpu ? "C" : "F");
            result = true;
          }
          else if (buf[idx] == 'C') {
            // Set temperature units
            cfgData.tmpu = 1;
            result = true;
          }
          else if (buf[idx] == 'F') {
            // Set temperature units
            cfgData.tmpu = 0;
            result = true;
          }
          else {
            // Get the digit value
            value = getValidDigit(buf, idx, 0, 1, HAYES_NUM_ERROR);
            if (value != HAYES_NUM_ERROR) {
              // Set temperature units
              cfgData.tmpu = value;
              result = true;
            }
          }
          break;

        // Supply voltage correction factor
        case 'V':
          if (len == idx or buf[idx] == '?') {
            Serial.print(F("*V: ")); Serial.println(cfgData.kvcc);
            result = true;
          }
          else {
            // Get the integer value
            value = getValidInteger(buf, idx, -127, 127, HAYES_NUM_ERROR);
            if (value != HAYES_NUM_ERROR) {
              // Set the voltage correction factor
              cfgData.kvcc = value;
              result = true;
            }
          }
          break;
      }
      break;

    // Standard '&' extension
    case '&':
      switch (buf[idx++]) { // idx++ -> 2
        // Factory defaults
        case 'F':
          result = cfgDefaults();
          break;

        // Show the configuration
        case 'V':
          Serial.print(F("*A: "));  Serial.print(cfgData.aubr); Serial.print(F("; "));
          Serial.print(F("*B: "));  Serial.print(cfgData.brgt); Serial.print(F("; "));
          Serial.print(F("*L: "));  Serial.print(cfgData.mnbr); Serial.print(F("; "));
          Serial.print(F("*H: "));  Serial.print(cfgData.mxbr); Serial.println(F("; "));
          Serial.print(F("*F: "));  Serial.print(cfgData.font); Serial.print(F("; "));
          Serial.print(F("*D: "));  Serial.print(cfgData.dst);  Serial.print(F("; "));
          Serial.print(F("*O: "));  Serial.print(mtxMode);      Serial.print(F("; "));
          Serial.print(F("*U: "));  Serial.print(cfgData.tmpu ? "C" : "F"); Serial.println(F("; "));
          Serial.print(F("*S: "));  Serial.print(cfgData.bfst); Serial.print(F("; "));
          Serial.print(F("*E: "));  Serial.print(cfgData.blst); Serial.print(F("; "));
          Serial.print(F("*M: "));  Serial.print(cfgData.ktmp); Serial.print(F("; "));
          Serial.print(F("*V: "));  Serial.print(cfgData.kvcc); Serial.println(F("; "));
          Serial.print(F("E: "));   Serial.print(cfgData.echo); Serial.print(F("; "));
          Serial.print(F("L: "));   Serial.print(cfgData.spkl); Serial.print(F("; "));
          Serial.print(F("M: "));   Serial.print(cfgData.spkm); Serial.print(F("; "));
          Serial.print(F("Q: "));   Serial.print(cfgData.scqt); Serial.println(F("; "));
          result = true;
          break;

        // Store the configuration
        case 'W':
          result = cfgWriteEE();
          break;

        // Read the configuration
        case 'Y':
          result = cfgReadEE();
          break;
      }
      break;

    // ATE Set local echo
    case 'E':
      if (buf[idx] == '?') {
        // Get local echo
        Serial.print(F("E: ")); Serial.println(cfgData.echo);
        result = true;
      }
      else if (len == idx) {
        cfgData.echo = 0x00;
        result = true;
      }
      else {
        // Get the digit value
        value = getValidDigit(buf, idx, 0, 1, HAYES_NUM_ERROR);
        if (value != HAYES_NUM_ERROR) {
          // Set echo on or off
          cfgData.echo = value;
          result = true;
        }
      }
      break;

    // ATI Show info
    case 'I': {
        uint8_t rqInfo = 0x00;
        if (len == idx or buf[idx] == '0') {
          // Display all info
          rqInfo = 0x03;
          result = true;
        }
        else {
          // Get the digit value
          value = getValidDigit(buf, idx, 1, 7, HAYES_NUM_ERROR);
          if (value != HAYES_NUM_ERROR) {
            // Specify the line to display
            rqInfo = 0x01 << (value - 1);
            result = true;
          }
        }
        if (result) {
          if (rqInfo & 0x01) print_P(DEVNAME, true);  rqInfo = rqInfo >> 1;
          if (rqInfo & 0x01) print_P(VERSION, true);  rqInfo = rqInfo >> 1;
          if (rqInfo & 0x01) print_P(AUTHOR,  true);  rqInfo = rqInfo >> 1;
          if (rqInfo & 0x01) print_P(DATE,    true);  rqInfo = rqInfo >> 1;
          if (rqInfo & 0x01) Serial.println(cfgData.crc8, 16);  rqInfo = rqInfo >> 1;
        }
      }
      break;

    // ATL Set speaker volume level
    case 'L':
      if (buf[idx] == '?') {
        // Get speaker volume level
        Serial.print(F("L: ")); Serial.println(cfgData.spkl);
        result = true;
      }
      else if (len == idx) {
        cfgData.spkl = 0x00;
        result = true;
      }
      else {
        // Get the digit value
        value = getValidDigit(buf, idx, 0, 3, HAYES_NUM_ERROR);
        if (value != HAYES_NUM_ERROR) {
          // Set speaker volume
          cfgData.spkl = value;
          result = true;
        }
      }
      break;

    // ATM Speaker control
    case 'M':
      if (buf[idx] == '?') {
        // Get speaker mode
        Serial.print(F("M: ")); Serial.println(cfgData.spkm);
        result = true;
      }
      else if (len == idx) {
        cfgData.spkm = 0x00;
        result = true;
      }
      else {
        // Get the digit value
        value = getValidDigit(buf, idx, 0, 3, HAYES_NUM_ERROR);
        if (value != HAYES_NUM_ERROR) {
          // Set speaker on or off mode
          cfgData.spkm = value;
          result = true;
        }
      }
      break;

    // ATQ Quiet Mode
    case 'Q':
      if (buf[idx] == '?') {
        // Get quiet mode
        Serial.print(F("Q: ")); Serial.println(cfgData.scqt);
        result = true;
      }
      else if (len == idx) {
        cfgData.scqt = 0x00;
        result = true;
      }
      else {
        // Get the digit value
        value = getValidDigit(buf, idx, 0, 1, HAYES_NUM_ERROR);
        if (value != HAYES_NUM_ERROR) {
          // Set console quiet mode off or on
          cfgData.scqt = value;
          result = true;
        }
      }
      break;

    // ATZ Reset
    case 'Z':
      softReset(WDTO_2S);
      break;

    // Help messages
    case '?':
      Serial.println(F("Set local echo              En    0..1"));
      Serial.println(F("Show info                   In    0..7"));
      Serial.println(F("Speaker volume level        Ln    0..3"));
      Serial.println(F("Speaker control             Mn    0..3"));
      Serial.println(F("Quiet Mode                  Qn    0..1"));
      Serial.println(F("Reset                       Z"));

      Serial.println(F("Load factory defaults       &F"));
      Serial.println(F("Show the configuration      &V"));
      Serial.println(F("Store the configuration     &W"));
      Serial.println(F("Read the configuration      &Y"));

      Serial.println(F("Auto brightness             *An   0..1"));
      Serial.println(F("Brightness level            *Bn   0..15"));
      Serial.println(F("DST switch                  *Dn   0..1"));
      Serial.println(F("Latest hour to beep         *En   0..23"));
      Serial.println(F("Display font                *Fn   0..15"));
      Serial.println(F("Maximum auto brightness     *Hn   0..15"));
      Serial.println(F("Lowest auto brightness      *Ln   0..15"));
      Serial.println(F("MCU temperature correction  *Mn   -127..127   T+273.15-ADC"));
      Serial.println(F("Display mode selection      *On   0..15       HHMM,SS,DDMM,YY,TMP,VCC,MCU,SET_TIME,SET_DATE"));
      Serial.println(F("First hour to beep          *Sn   0..23"));
      Serial.println(F("Time and date setting       *T=\"YYYY/MM/DD HH:MM:SS\""));
      Serial.println(F("Temperature units           *Uc   C/F"));
      Serial.println(F("Supply voltage correction   *Vn   -127..127   V*ADC/1.1/1024-1000"));
      result = true;
      break;

    default:
      result = (len == 0);
  }

  // Last response line
  if (len >= 0) {
    if (result) Serial.println(F("OK"));
    else        Serial.println(F("ERROR"));

    // Force time display
    mtxDisplayNow = true;
    Serial.flush();
  }
}

/**
  Check the DST adjustments
*/
bool checkDST() {
  int8_t dstAdj = rtc.dstSelfAdjust(cfgData.dst);
  if (dstAdj != 0) {
    // Do the adjustment, no need to re-read the RTC
    rtc.setHours(dstAdj, false);
    // Toggle the DST config bit
    cfgData.dst ^= 1;
    // Save the config
    cfgWriteEE();
    return true;
  }
  return false;
}

/**
  Print the banner to serial console
*/
void showBanner() {
  print_P(DEVNAME);
  Serial.print(F(" "));
  print_P(VERSION, true);
}

/**
  Software reset the MCU
  (c) Mircea Diaconescu http://web-engineering.info/node/29
*/
void softReset(uint8_t prescaller) {
  Serial.print(F("Reboot"));
  // Start watchdog with the provided prescaller
  wdt_enable(prescaller);
  // Wait for the prescaller time to expire
  // without sending the reset signal by using
  // the wdt_reset() method
  while (true) {
    Serial.print(F("."));
    delay(1000);
  }
}

/**
  Main Arduino setup function
*/
void setup() {
  // Init the serial com and print the banner
  Serial.begin(9600);
  showBanner();
  // Make us heard with a beep
  beep();

  // Init the buttons
  btn1.begin();
  btn2.begin();
  intsq.begin();

  // Read the configuration from EEPROM or
  // use the defaults if CRC8 does not match
  cfgReadEE(true);

  // Start with a dummy analog read
  analogRead(A0);

  // Init all led matrices
  mtx.init(CS_PIN, MATRICES, SCANLIMIT);
  // Do a display test for a second
  mtx.displaytest(true);
  delay(1000);
  mtx.displaytest(false);
  // Decode nothing
  mtx.decodemode(0);
  // Clear the display
  mtx.clear();
  // Set the brightness
  mtx.intensity(brightness());
  brgtCheckUntil = millis();
  // Power on the matrices
  mtx.shutdown(false);

  // Load the font
  mtx.loadFont(cfgData.font);

  // Show version
  showModeVers();

  // Init and configure RTC
  if (! rtc.init()) {
    Serial.print(F("ERROR"));
    Serial.println(F(" RTC missing"));
  }

  // Only if RTC is present and working
  if (rtc.rtcOk) {
    // Check the power
    if (rtc.lostPower()) {
      Serial.print(F("ERROR"));
      Serial.println(F(" RTC power lost"));
    };

    // Check DST adjustments
    checkDST();
  }

  // Start reading the remote
  if (!iRed.begin(IRED_PIN))
    Serial.println(F("You did not choose a valid IR pin."));

  // Wait a second while displaying the verion
  delay(1000);
}

/**
  Main Arduino loop
*/
void loop() {
  // Check if new IR data is available
  if (iRed.available()) {
    // Get the new data from the remote
    auto data = iRed.read();
    if (data.address == 0x728D) // Akai CD
      switch (data.command) {
        case 0:
        case 1:
        case 2:
        case 3:
        case 4:
        case 5:
        case 6:
        case 7:
        case 8:
        case 9:
          mtxSetMode(data.command);
          break;
        case 0x1D:  // Next
          if (++cfgData.font > fontCount)
            cfgData.font = 0;
          mtx.loadFont(cfgData.font);
          mtxDisplayNow = true;
          break;
        case 0x1E:  // Prev
          if (cfgData.font-- == 0)
            cfgData.font = fontCount;
          mtx.loadFont(cfgData.font);
          mtxDisplayNow = true;
          break;
        case 0x16:  // Prg up
          if (cfgData.brgt <= 0x0F)
            cfgData.brgt++;
          cfgData.aubr = false;
          mtx.intensity(brightness());
          break;
        case 0x17:  // Prg dn
          if (cfgData.brgt >= 0x00)
            cfgData.brgt--;
          cfgData.aubr = false;
          mtx.intensity(brightness());
          break;
        case 0x0B:  // Mute
          cfgData.aubr ^= 1;
          mtx.intensity(brightness());
          break;
      }
    else if (data.address == 0xFB04) // LG Game Remote
      switch (data.command) {
        case 0x06:  // Right
          mtxNextMode();
          break;
        case 0x07:  // Left
          mtxPrevMode();
          break;
        case 0x7C:  // Home
          mtxSetMode(MODE_HHMM);
          mtx.intensity(cfgDefault.brgt);
          mtx.loadFont(cfgDefault.font);
          mtxDisplayNow = true;
          break;
        case 0xE4:  // Game
          cfgReadEE();
          mtxSetMode(MODE_HHMM);
          mtx.intensity(cfgData.brgt);
          mtx.loadFont(cfgData.font);
          mtxDisplayNow = true;
          break;
        case 0x00:  // Prg up
          if (++cfgData.font > fontCount)
            cfgData.font = 0;
          mtx.loadFont(cfgData.font);
          mtxDisplayNow = true;
          break;
        case 0x01:  // Prg dn
          if (cfgData.font-- == 0)
            cfgData.font = fontCount;
          mtx.loadFont(cfgData.font);
          mtxDisplayNow = true;
          break;
        case 0x02:  // Vol up
          if (cfgData.brgt <= 0x0F)
            cfgData.brgt++;
          cfgData.aubr = false;
          mtx.intensity(brightness());
          break;
        case 0x03:  // Vol dn
          if (cfgData.brgt >= 0x00)
            cfgData.brgt--;
          cfgData.aubr = false;
          mtx.intensity(brightness());
          break;
        case 0x0B:  // Source
          cfgData.aubr ^= 1;
          mtx.intensity(brightness());
          break;
        case 0x44:  // OK
          beep();
          break;
      }
    if (data.address != 0xFFFF) {
      // Print the protocol data
      Serial.print(F("Address: 0x"));
      Serial.println(data.address, HEX);
      Serial.print(F("Command: 0x"));
      Serial.println(data.command, HEX);
      Serial.println();
    }
  }

  // Check any command on serial port
  if (Serial.available())
    handleHayes();

  // Keep the millis
  uint32_t now = millis();

  // Automatic brightness check and adjustment
  if (cfgData.aubr and (now > brgtCheckUntil)) {
    brgtCheckUntil = now + brgtCheckWait;
    mtx.intensity(brightness());
  }

  // Check the buttons and change the display mode
  if (btn1.pressed()) {
    // If we're in time/date setting mode, adjust values
    if (mtxMode == MODE_SET_TIME) {
      // Increment the currently selected time field
      if (setTimeField == 0) {
        // Adjust hours with wraparound (0-23)
        setHours = (setHours + 1) % 24;
      } else {
        // Adjust minutes with wraparound (0-59)
        setMinutes = (setMinutes + 1) % 60;
      }
      showSetTime(setHours, setMinutes);
    } else if (mtxMode == MODE_SET_DATE) {
      // Increment the currently selected date field
      if (setDateField == 0) {
        // Adjust day with simple wraparound (1-31)
        setDay = (setDay % 31) + 1;
      } else if (setDateField == 1) {
        // Adjust month with wraparound (1-12)
        setMonth = (setMonth % 12) + 1;
      } else {
        // Adjust year with bounds checking (2000-2099)
        setYear = (setYear < 2099) ? setYear + 1 : 2000;
      }
      showSetDate(setDay, setMonth, setYear);
    } else {
      // Display the next mode in normal operation
      mtxNextMode();
    }
  }

  if (btn2.pressed()) {
    // If we're in time/date setting mode, switch fields or save
    if (mtxMode == MODE_SET_TIME) {
      if (setTimeField == 0) {
        // Switch from hours to minutes field
        setTimeField = 1;
      } else {
        // Save time and return to normal mode
        rtc.readTime(true); // Get current date to preserve it
        rtc.writeDateTime(0, setMinutes, setHours, rtc.d, rtc.m, rtc.Y);
        mtxSetMode(MODE_HHMM);
      }
      showSetTime(setHours, setMinutes);
    } else if (mtxMode == MODE_SET_DATE) {
      if (setDateField < 2) {
        // Switch to next field (day -> month -> year)
        setDateField++;
      } else {
        // Save date and return to normal mode
        rtc.writeDateTime(rtc.S, rtc.M, rtc.H, setDay, setMonth, setYear);
        mtxSetMode(MODE_HHMM);
      }
      showSetDate(setDay, setMonth, setYear);
    } else if (mtxMode == MODE_HHMM) {
      // Enter time setting mode from HHMM display
      rtc.readTime(true);
      setHours = rtc.H;
      setMinutes = rtc.M;
      setTimeField = 0; // Start with hours field
      mtxSetMode(MODE_SET_TIME);
      showSetTime(setHours, setMinutes);
    } else if (mtxMode == MODE_DDMM) {
      // Enter date setting mode from DDMM display
      rtc.readTime(true);
      setDay = rtc.d;
      setMonth = rtc.m;
      setYear = rtc.Y;
      setDateField = 0; // Start with day field
      mtxSetMode(MODE_SET_DATE);
      showSetDate(setDay, setMonth, setYear);
    } else {
      // Display the previous mode in normal operation
      mtxPrevMode();
    }
  }

  // Display, check once in a while or force
  if ((now > mtxDisplayUntil) or mtxDisplayNow) {
    mtxDisplayUntil = now + mtxDisplayWait;
    switch (mtxMode) {
      case MODE_SS:   // Seconds
        showModeSS();
        break;
      case MODE_DDMM: // Day and month
        showModeDDMM();
        break;
      case MODE_YY:   // Year
        showModeYY();
        break;
      case MODE_TEMP: // RTC temperature
        showModeTEMP();
        break;
      case MODE_VCC:  // Power supply voltage
        showModeVCC();
        break;
      case MODE_MCU:  // MCU temperature
        showModeMCU();
        break;
      case MODE_SET_TIME:  // Set time
        showSetTime(setHours, setMinutes);
        break;
      case MODE_SET_DATE:  // Set date
        showSetDate(setDay, setMonth, setYear);
        break;
      default:        // Hours and minutes
        showModeHHMM();
    }
    // Reset the display now flag
    mtxDisplayNow = false;
  }

  // Check if the display mode expired
  if (mtxModeUntil > 0 and now > mtxModeUntil)
    // Return to default mode, never expiring
    mtxSetMode(MODE_HHMM);
}
