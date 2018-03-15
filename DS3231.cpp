/**
  DS3231.cpp - Simple DS3231 library

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

#include <Arduino.h>
#include <Wire.h>
#include "DS3231.h"

// Convert to binary coded decimal
uint8_t bin2bcd(uint8_t x) {
  return (x / 10 * 16) + (x % 10);
}

// Convert from binary coded decimal
uint8_t bcd2bin(uint8_t x) {
  return (x / 16 * 10) + (x % 16);
}


DS3231::DS3231() {
}

bool DS3231::init(uint8_t twRTC, bool twInit) {
  // Init i2c
  if (twInit) Wire.begin();
  // Keep the address
  rtcAddr = twRTC;
  // Check the device is present
  Wire.beginTransmission(rtcAddr);
  rtcOk = Wire.endTransmission() == 0;
  // Set century
  C = CENTURY * 100;

  /*
    // Debug: show all registers in DS3231
    Wire.beginTransmission(rtcAddr);
    Wire.write(0x00);
    Wire.endTransmission();
    // Request all bytes of data starting from 0x01
    Wire.requestFrom(rtcAddr, (uint8_t)0x12);
    char buf[16];
    for (uint8_t i = 0x00; i <= 0x12; i++) {
      uint8_t x = Wire.read();
      sprintf(buf, "%02x: %02x", i, x);
      Serial.println(buf);
    }
  */

  if (rtcOk) {
    // Set Alarm 2 to trigger every minute
    Wire.beginTransmission(rtcAddr);
    // Alarm 2 minutes register
    Wire.write(0x0B);
    Wire.write(0x80); // 0x0B
    Wire.write(0x80); // 0x0C
    Wire.write(0x80); // 0x0D
    Wire.endTransmission();

    // Check the control register
    Wire.beginTransmission(rtcAddr);
    // Control Register
    Wire.write(0x0E);
    Wire.endTransmission();
    // Request one byte
    Wire.requestFrom(rtcAddr, (uint8_t)1);
    uint8_t x = Wire.read();
    // Enable Alarm 2 INT
    Wire.beginTransmission(rtcAddr);
    // Control Register
    Wire.write(0x0E);
    // Set INTCN and A2IE bits
    Wire.write(x | 0x06);
    Wire.endTransmission();
  }
  return rtcOk;
}

/**
  Read the current time from the RTC and unpack it into the object variables

  @return true if the function succeeded
*/
bool DS3231::readTime(bool readDate) {
  Wire.beginTransmission(rtcAddr);
  // Seconds register
  Wire.write(0x00);
  if (Wire.endTransmission() != 0)
    return false;
  // Request 3 or 7 bytes of data starting from 0x00
  uint8_t len = 3;
  if (readDate) len = 7;
  Wire.requestFrom(rtcAddr, len);

  // Seconds
  S = Wire.read();
  S = bcd2bin(S);
  // Minutes
  M = Wire.read();
  M = bcd2bin(M);
  // Hours
  H = Wire.read();
  // Check if 12 hours and PM and add 0x12 (BCD)
  if ((H & (1 << 6)) and (H & (1 << 5)))
    H = (H & 0x1F) + 0x12;
  H = bcd2bin(H & 0x3F);
  P = H >= 12;
  if (H > 12) I = H - 12;
  else        I = H;
  // Date
  if (readDate) {
    // Century
    uint16_t c = C;
    // Day of week, 1 is Monday
    u = Wire.read();
    u = bcd2bin(u & 0x07);
    // Day
    d = Wire.read();
    d = bcd2bin(d);
    // Month and century
    m = Wire.read();
    if (m & (1 << 7)) c += 100;
    m = bcd2bin(m & 0x1F);
    // Year, short and long format
    y = Wire.read();
    y = bcd2bin(y);
    Y = c + y;
  }
  return true;
}

/**
  Read the current time from the RTC as unpacked BCD in
  preallocated buffer

  @return true if the function succeeded
*/
bool DS3231::readTimeBCD() {
  Wire.beginTransmission(rtcAddr);
  // Minute register
  Wire.write(0x01);
  if (Wire.endTransmission() != 0)
    return false;
  // Request 3 bytes of data starting from 0x01
  Wire.requestFrom(rtcAddr, (uint8_t)3);

  uint8_t x;
  // Minutes
  x = Wire.read();
  R[2] = x / 16; (x & 0xF0) >> 4;
  R[3] = x % 16; (x & 0x0F);
  // Hours
  x = Wire.read() & 0x3F;
  R[0] = x / 16; (x & 0xF0) >> 4;
  R[1] = x % 16; (x & 0x0F);
  return true;
}

/**
  Read the current temperature from the RTC

  @return integer temperature
*/
int8_t DS3231::readTemperature(bool metric) {
  Wire.beginTransmission(rtcAddr);
  // Temperature register
  Wire.write(0x11);
  if (Wire.endTransmission() != 0)
    return 0x80;
  // Request one byte
  Wire.requestFrom(rtcAddr, (uint8_t)1);
  int8_t t = Wire.read();
  if (not metric) t = (int8_t)((float)t * 1.8 + 32.0);
  return t;
}

/**
  Read the status bit of the RTC

  @return bool bit status
*/
bool DS3231::lostPower() {
  Wire.beginTransmission(rtcAddr);
  // Status Register
  Wire.write(0x0F);
  if (Wire.endTransmission() != 0)
    return true;
  // Request one byte
  Wire.requestFrom(rtcAddr, (uint8_t)1);
  uint8_t x = Wire.read();
  // Return the MSB
  return x & 0x80;
}

/**
  Check and clear the alarms

  @return triggered alarm
*/
uint8_t DS3231::checkAlarms() {
  Wire.beginTransmission(rtcAddr);
  // Status Register
  Wire.write(0x0F);
  if (Wire.endTransmission() != 0)
    return false;
  // Request one byte
  Wire.requestFrom(rtcAddr, (uint8_t)1);
  uint8_t x = Wire.read();
  uint8_t result = x & 0x03;
  if (result) {
    // Clear the alarms
    Wire.beginTransmission(rtcAddr);
    // Status Register
    Wire.write(0x0F);
    // Clear the two less significant bits
    Wire.write(x & 0xFC);
    Wire.endTransmission();
  }
  return result;
}

/**
   Set RTC date and time and clear the status flag

   @param uint8_t S second to set to HW RTC
   @param uint8_t M minute to set to HW RTC
   @param uint8_t H hour to set to HW RTC
   @param uint8_t d day of month to set to HW RTC
   @param uint8_t m month to set to HW RTC
   @param uint8_t Y year to set to HW RTC
*/
bool DS3231::writeDateTime(uint8_t S, uint8_t M, uint8_t H,
                           uint8_t d, uint8_t m, uint16_t Y) {
  // Compute the day of the week, format 1..7, Monday based
  uint8_t u = getDOW(Y, m, d);
  if (u == 0) u = 7;
  // Century flag
  uint8_t c = 0x00;
  // Set DS3231 register pointer to 0x00
  Wire.beginTransmission(rtcAddr);
  Wire.write(0x00);
  Wire.write(bin2bcd(S % 60));        // Seconds, 00..59
  Wire.write(bin2bcd(M % 60));        // Minutes, 00..59
  Wire.write(bin2bcd(H % 24) & 0x3F); // Hours, 00..23
  Wire.write(u);                      // Day of week, Mon first
  Wire.write(bin2bcd(d % 31) & 0x3F); // Day in month, 01..31
  if (Y > (C + 99)) c = 1 << 7;
  Wire.write(bin2bcd(m % 12) + c);    // Month, 01..12, and century flag
  Wire.write(bin2bcd(Y % 100));       // Year, 00..99
  Wire.endTransmission();

  // Clear the status flag
  Wire.beginTransmission(rtcAddr);
  Wire.write(0x0F);
  Wire.endTransmission();
  // Request one byte
  Wire.requestFrom(rtcAddr, (uint8_t)1);
  uint8_t x = Wire.read();
  // Clear the status bit
  Wire.beginTransmission(rtcAddr);
  Wire.write(0x0F);
  Wire.write(x & 0x7F);
  Wire.endTransmission();
}

/**
   Reset RTC senconds to zero
*/
bool DS3231::resetSeconds() {
  // Set DS3231 register pointer to 0x00
  S = 0;
  Wire.beginTransmission(rtcAddr);
  Wire.write(0x00);
  Wire.write(S);
  return (Wire.endTransmission() == 0);
}

/**
   Increment the minutes
*/
bool DS3231::incMinutes() {
  // Read the time
  readTime();
  // Increment the minutes
  M = (M + 1) % 60;
  // Set DS3231 register pointer to 0x01
  Wire.beginTransmission(rtcAddr);
  Wire.write(0x01);
  Wire.write(bin2bcd(M));
  return (Wire.endTransmission() == 0);
}

/**
   Increment the hours
*/
bool DS3231::incHours() {
  // Read the time
  readTime();
  // Increment the hours
  H = (H + 1) % 24;
  P = H >= 12;
  if (H > 12) I = H - 12;
  else        I = H;
  // Set DS3231 register pointer to 0x02
  Wire.beginTransmission(rtcAddr);
  Wire.write(0x02);
  Wire.write(bin2bcd(H & 0x3F));
  return (Wire.endTransmission() == 0);
}

/**
  Determine the day of the week using the Tomohiko Sakamoto's method

  @param y year  >1752
  @param m month 1..12
  @param d day   1..31
  @return day of the week, 0..6 (Sun..Sat)
*/
uint8_t DS3231::getDOW(uint16_t year, uint8_t month, uint8_t day) {
  uint8_t t[] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};
  year -= month < 3;
  return (year + year / 4 - year / 100 + year / 400 + t[month - 1] + day) % 7;
}

/**
  Check if a specified date observes DST, according to
  the time changing rules in Europe:

    start: last Sunday in March
    end:   last Sunday in October

  @param year  year  >1752
  @param month month 1..12
  @param day   day   1..31
  @return bool DST yes or no
*/
bool DS3231::isDST(uint16_t year, uint8_t month, uint8_t day) {
  // Get the last Sunday in March
  uint8_t dayBegin = 31 - getDOW(year, 3, 31);
  // Get the last Sunday on October
  uint8_t dayEnd = 31 - getDOW(year, 10, 31);
  // Compute DST
  return ((month > 3) and (month < 10)) or
         ((month == 3) and (day >= dayBegin)) or
         ((month == 10) and (day < dayEnd));
}



