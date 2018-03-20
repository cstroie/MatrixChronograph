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

#include "DotMatrix.h"
#include "DS3231.h"

const char DEVNAME[] = "LedMatrix Clock";
const char VERSION[] = "2.4";

// Pin definitions
const int CS_PIN    = 10; // ~SS
const int BEEP_PIN  = 2;

// The RTC
DS3231 rtc;
uint32_t rtcDelayCheck  = 1000UL;
uint32_t rtcLastCheck   = 0UL;

// The matrix object
#define MATRICES  4
#define SCANLIMIT 8
DotMatrix mtx = DotMatrix();


// Define the configuration type
struct cfgEE_t {
  union {
    struct {
      uint8_t font: 4;  // Display font
      uint8_t brgt: 4;  // Display brightness (manual)
      uint8_t mnbr: 4;  // Minimal display brightness (auto)
      uint8_t mxbr: 4;  // Minimum display brightness (auto)
      bool    aubr: 1;  // Manual/Auto display brightness adjustment
      bool    tmpu: 1;  // Temperature units
      uint8_t spkm: 2;  // Speaker mode
      uint8_t spkl: 2;  // Speaker volume level
      uint8_t echo: 1;  // Local echo
      bool    dst:  1;  // DST flag
    };
    uint8_t data[4];    // We use 4 bytes in the structure
  };
  uint8_t crc8;         // CRC8
};
// The global configuration structure
struct cfgEE_t  cfgData;
// EEPROM address to store the configuration to
uint16_t        cfgEEAddress = 0x0180;

// Show time independently of minutes changing
bool mtxShowTime = true;


/**
  CRC8 computing

  @param inCrc partial CRC
  @param inData new data
  @return updated CRC
*/
uint8_t crc_8(uint8_t inCrc, uint8_t inData) {
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
    crc8 = crc_8(crc8, cfgData.data[i]);
  return crc8;
}

/**
  Compare two configuration structures

  @param cfg1 the first configuration structure
  @param cfg1 the second configuration structure
  @return true if equal
*/
bool cfgCompare(struct cfgEE_t cfg1, struct cfgEE_t cfg2) {
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
void cfgWriteEE() {
  // Temporary configuration structure
  struct cfgEE_t cfgTemp;
  // Read the data from EEPROM
  EEPROM.get(cfgEEAddress, cfgTemp);
  // Compute the CRC8 checksum of the current data
  cfgData.crc8 = cfgEECRC(cfgData);
  // Compare the new and the stored data
  if (not cfgCompare(cfgData, cfgTemp))
    // Write the data
    EEPROM.put(cfgEEAddress, cfgData);
}

/**
  Read the configuration from EEPROM, along with CRC8, and verify
*/
bool cfgReadEE(bool useDefaults = false) {
  // Temporary configuration structure
  struct cfgEE_t cfgTemp;
  // Read the data from EEPROM
  EEPROM.get(cfgEEAddress, cfgTemp);
  // Compute the CRC8 checksum of the read data
  uint8_t crc8 = cfgEECRC(cfgTemp);
  // Check if the read data is valid
  if      (cfgTemp.crc8 == crc8)  cfgData = cfgTemp;
  else if (useDefaults)           cfgDefaults();
  return (cfgTemp.crc8 == crc8);
}

/**
  Reset the configuration to factory defaults
*/
bool cfgDefaults() {
  cfgData.font = 0x01;
  cfgData.brgt = 0x01;
  cfgData.mnbr = 0x00;
  cfgData.mxbr = 0x0F;
  cfgData.aubr = 0x01;
  cfgData.tmpu = 0x01;
  cfgData.spkm = 0x00;
  cfgData.spkl = 0x00;
  cfgData.echo = 0x01;
  cfgData.dst  = 0x00;
};

/**
  Compute the brightness
*/
uint8_t brightness() {
  if (cfgData.aubr) {
    // Automatic, read the LDR connected to A0
    uint16_t light = analogRead(A0);
    return map(light, 0, 1023, cfgData.mnbr, cfgData.mxbr);
  }
  else
    // Manual
    return cfgData.brgt;
}

/**
  Show the time specfied in unpacked BCD (4 bytes)
*/
void showTimeBCD(uint8_t* HHMM) {
  // Create a new array, containing the colon symbol
  uint8_t HH_MM[] = {HHMM[0], HHMM[1], 0x0A, HHMM[2], HHMM[3]};

  // Digits positions and count
  uint8_t pos[] = {23, 17, 13, 9, 3};
  uint8_t posCount = sizeof(pos) / sizeof(*pos);

  // Clear the framebuffer
  mtx.fbClear();

  // Print on framebuffer
  for (uint8_t d = 0; d < posCount; d++)
    mtx.fbPrint(pos[d], HH_MM[d]);

  // Display the framebuffer
  mtx.fbDisplay();
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
  Show the temperature
*/
void showTemperature() {
  // Get the temperature
  int8_t temp = rtc.readTemperature(cfgData.tmpu);

  // Create a new array, containing the degree symbol and units letter
  uint8_t data[] = {0x0B, abs(temp) / 10, abs(temp) % 10, 0x0D, cfgData.tmpu ? 0x0C : 0x0F};

  // Digits positions and count
  uint8_t pos[] = {27, 21, 15, 9, 3};
  uint8_t posCount = sizeof(pos) / sizeof(*pos);

  // Clear the framebuffer
  mtx.fbClear();

  // Print into the framebuffer
  for (uint8_t d = temp < 0 ? 0 : 1; d < posCount; d++)
    mtx.fbPrint(pos[d], data[d]);

  // Display the framebuffer
  mtx.fbDisplay();
}

/**
  Short BEEP

  @param duration beep duration in ms
*/
void beep(uint16_t duration = 20) {
  pinMode(BEEP_PIN, OUTPUT);
  digitalWrite(BEEP_PIN, HIGH);
  delay(duration);
  digitalWrite(BEEP_PIN, LOW);
}

/**
  AT-Hayes style command processing
*/
void handleHayes() {
  char buf[35] = "";
  int8_t len = -1;
  bool result = false;
  if (Serial.find("AT")) {
    // Read until newline, no more than 4 chararcters
    len = Serial.readBytesUntil('\r', buf, 4);
    buf[len] = '\0';
    // Local echo
    if (cfgData.echo) {
      Serial.print(F("AT"));
      Serial.println(buf);
    }
    // Check the first character, could be a symbol or a letter
    if      (buf[0] == '*') {
      // Our extension, one char and numeric value (0-99)
      if      (buf[1] == 'F') {
        // Font
        if (buf[2] >= '0' and buf[2] <= '9') {
          // Set font
          uint8_t font = buf[2] - '0';
          // Read one more char
          if (buf[3] >= '0' and buf[3] <= '9')
            font = (font * 10 + (buf[3] - '0')) % fontCount;
          cfgData.font = font;
          mtx.loadFont(cfgData.font);
          result = true;
        }
        else if (buf[2] == '?') {
          // Get font
          Serial.print(F("*F: "));
          Serial.println(cfgData.font);
          result = true;
        }
      }
      else if (buf[1] == 'B') {
        // Brightness
        if (buf[2] >= '0' and buf[2] <= '9') {
          // Set brightness
          uint8_t brgt = buf[2] - '0';
          // Read one more char
          if (buf[3] >= '0' and buf[3] <= '9')
            brgt = (brgt * 10 + (buf[3] - '0')) % 0x10;
          // Reset brightness manually
          cfgData.aubr = false;
          cfgData.brgt = brgt;
          // Set the brightness
          mtx.intensity(cfgData.brgt);
          result = true;
        }
        else if (buf[2] == '?') {
          // Get brightness
          Serial.print(F("*B: "));
          Serial.println(cfgData.brgt);
          result = true;
        }
      }
      else if (buf[1] == 'U') {
        // Temperature
        if (buf[2] >= '0' and buf[2] <= '1') {
          // Set temperature units
          cfgData.tmpu = buf[2] == '1' ? true : false;
          result = true;
        }
        else if (buf[2] == '?') {
          // Get temperature units
          Serial.print(F("*U: "));
          Serial.println(cfgData.tmpu ? "C" : "F");
          result = true;
        }
        else if (len == 2) {
          // Show the temperature
          Serial.print((int)rtc.readTemperature(cfgData.tmpu));
          Serial.println(cfgData.tmpu ? "C" : "F");
          result = true;
        }
      }
      else if (buf[1] == 'T') {
        // Time and date
        //  Usage: AT*T="YYYY/MM/DD HH:MM:SS" or, using date(1),
        //  ( sleep 2 && date "+AT\*T=\"%Y/%m/%d %H:%M:%S\"" ) > /dev/ttyUSB0
        if (buf[2] == '=' and buf[3] == '"') {
          // Set time and date
          uint16_t  year  = Serial.parseInt();
          uint8_t   month = Serial.parseInt();
          uint8_t   day   = Serial.parseInt();
          uint8_t   hour  = Serial.parseInt();
          uint8_t   min   = Serial.parseInt();
          uint8_t   sec   = Serial.parseInt();
          if (year != 0 and month != 0 and day != 0) {
            // The date is quite valid, set the clock to 00:00:00 if not
            // specified
            rtc.writeDateTime(sec, min, hour, day, month, year);
            // Check if DST and set the flag
            cfgData.dst = rtc.dstCheck(year, month, day, hour);
            // Store the configuration
            cfgWriteEE();
            // TODO Check
            result = true;
          }
        }
        else if (buf[2] == '?') {
          // TODO Get time and date
          result = true;
        }
      }
    }
    else if (buf[0] == '&') {
      // Standard '&' extension
      switch (buf[1]) {
        case 'F':
          // Factory defaults
          cfgDefaults();
          result = true;
          break;
        case 'V':
          // Show the configuration
          Serial.print(F("*F: "));
          Serial.println(cfgData.font);
          Serial.print(F("*B: "));
          Serial.println(cfgData.brgt);
          Serial.print(F("*U: "));
          Serial.println(cfgData.tmpu ? "C" : "F");
          result = true;
          break;
        case 'W':
          // Store the configuration
          cfgWriteEE();
          result = true;
          break;
        case 'Y':
          // Read the configuration
          result = cfgReadEE();
          break;
      }
    }
    else if (buf[0] == 'I' and len == 1) {
      // ATI
      showBanner();
      Serial.println(__DATE__);
      result = true;
    }
    else if (buf[0] == 'E') {
      // ATL Set local echo
      if (len == 1) {
        cfgData.echo = 0x00;
        result = true;
      }
      else if (buf[1] >= '0' and buf[1] <= '1') {
        // Set echo on or off
        cfgData.echo = (buf[1] - '0') & 0x01;
        result = true;
      }
      else if (buf[1] == '?') {
        // Get local echo
        Serial.print(F("E: "));
        Serial.println(cfgData.echo);
        result = true;
      }
    }
    else if (buf[0] == 'L') {
      // ATL Set speaker volume
      if (len == 1) {
        cfgData.spkl = 0x00;
        result = true;
      }
      else if (buf[1] >= '0' and buf[1] <= '3') {
        // Set speaker volume
        cfgData.spkl = (buf[1] - '0') & 0x03;
        result = true;
      }
      else if (buf[1] == '?') {
        // Get speaker volume level
        Serial.print(F("L: "));
        Serial.println(cfgData.spkl);
        result = true;
      }
    }
    else if (buf[0] == 'M') {
      // ATM Speaker control
      if (len == 1) {
        cfgData.spkm = 0x00;
        result = true;
      }
      else if (buf[1] >= '0' and buf[1] <= '3') {
        // Set speaker on or off mode
        cfgData.spkm = (buf[1] - '0') & 0x03;
        result = true;
      }
      else if (buf[1] == '?') {
        // Get speaker mode
        Serial.print(F("M: "));
        Serial.println(cfgData.spkm);
        result = true;
      }
    }
    else if (buf[0] == '?' and len == 1) {
      // Help message
      Serial.println(F("AT?"));
      Serial.println(F("AT*Fn"));
      Serial.println(F("AT*Bn"));
      Serial.println(F("AT*Un"));
      Serial.println(F("AT*T=\"YYYY/MM/DD HH:MM:SS\""));
      Serial.println(F("AT&F"));
      Serial.println(F("AT&V"));
      Serial.println(F("AT&W"));
      Serial.println(F("AT&Y"));
      result = true;
    }
    else if (len == 0)
      result = true;
  }

  if (len >= 0) {
    if (result)
      Serial.println(F("OK"));
    else
      Serial.println(F("ERROR"));

    // Force time display
    mtxShowTime = true;
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
  Serial.print(DEVNAME);
  Serial.print(" ");
  Serial.println(VERSION);
}

/**
  Main Arduino setup function
*/
void setup() {
  // Init the serial com and print the banner
  Serial.begin(9600);
  showBanner();

  // Read the configuration from EEPROM or
  // use the defaults if CRC8 does not match
  cfgReadEE(true);

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
  // FIXME Set the brightness
  mtx.intensity(brightness());
  // Power on the matrices
  mtx.shutdown(false);

  // Load the font
  mtx.loadFont(cfgData.font);

  // Init and configure RTC
  if (! rtc.init()) {
    Serial.println(F("DS3231 RTC missing"));
  }

  if (rtc.rtcOk and rtc.lostPower()) {
    Serial.println(F("RTC lost power, lets set the time!"));
    // The next line sets the RTC to the date & time this sketch was compiled
    //rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }

  if (rtc.rtcOk) {
    // Check DST adjustments
    checkDST();
    // Show the temperature
    showTemperature();
    delay(2000);
  }
}

/**
  Main Arduino loop
*/
void loop() {
  // Check any command on serial port
  if (Serial.available())
    handleHayes();

  if ((rtc.rtcOk and (millis() - rtcLastCheck > rtcDelayCheck)) or mtxShowTime) {
    rtcLastCheck = millis();
    // Check the alarms, the Alarm 2 triggers once per minute
    if ((rtc.checkAlarms() & 0x02) or mtxShowTime) {
      // Read the RTC, in BCD format
      bool newHour = rtc.readTimeBCD();
      // Check DST adjustments each new hour and read RTC if adjusted
      if (newHour)
        if (checkDST())
          rtc.readTimeBCD();
      // Show the time
      showTimeBCD(rtc.R);
      // Reset the forced show time flag
      mtxShowTime = false;
      // Beep each hour, on the dot
      if (newHour and (cfgData.spkm & 0x01)) beep();
    }
  }
}
