# MatrixChronograph

A desk clock using 4 8x8 LED matrix modules based on MAX7219 drivers.

## Features

- **Time Display**: Shows hours and minutes in various fonts
- **Date Display**: Shows day/month and year
- **Temperature Monitoring**: Displays ambient temperature from DS3231 RTC
- **Voltage Monitoring**: Shows MCU supply voltage
- **MCU Temperature**: Displays microcontroller temperature
- **Automatic Brightness**: Adjusts display brightness based on ambient light
- **DST Support**: Automatic Daylight Saving Time adjustments for Europe
- **IR Remote Control**: Control via infrared remote (Akai CD or LG Game Remote)
- **Button Control**: Navigate through display modes using physical buttons
- **Time Setting**: Set time and date using buttons or serial commands
- **Multiple Fonts**: 15 different fonts for display customization
- **Audio Feedback**: Configurable beeping on the hour
- **Serial Interface**: Hayes-compatible AT command interface for configuration

## Display Modes

1. **HHMM** - Hours and minutes (default)
2. **SS** - Seconds
3. **DDMM** - Day and month
4. **YY** - Year
5. **TEMP** - RTC temperature
6. **VCC** - Supply voltage
7. **MCU** - MCU temperature
8. **SET_TIME** - Time setting mode
9. **SET_DATE** - Date setting mode

## Hardware

- Arduino compatible microcontroller
- DS3231 Real-Time Clock with temperature sensor
- 4 x 8x8 LED matrix modules (MAX7219)
- Light Dependent Resistor (LDR) for automatic brightness
- Piezo buzzer for audio feedback
- Two push buttons for navigation
- Infrared receiver for remote control
- Optional: DS3231 interrupt pin connection

## Serial Commands

The device supports Hayes-compatible AT commands for configuration:

- `AT*O=n` - Set display mode (0-9)
- `AT*F=n` - Set font (0-14)
- `AT*B=n` - Set brightness (0-15)
- `AT*A=n` - Enable/disable automatic brightness (0-1)
- `AT*T="YYYY/MM/DD HH:MM:SS"` - Set time and date
- `AT*U=C/F` - Set temperature units
- `AT&Mn` - Set speaker mode (0-3)
- `AT&Ln` - Set speaker volume (0-3)
- `AT&Dn` - Enable/disable DST (0-1)
- `AT&V` - Show current configuration
- `AT&W` - Save configuration to EEPROM
- `AT&Y` - Load configuration from EEPROM
- `AT&F` - Load factory defaults

Type `AT?` for a full list of commands.

## Button Controls

- **Button 1**: Next display mode / Increment value in setting mode
- **Button 2**: Previous display mode / Enter setting mode / Confirm settings
  - Press on HHMM display to enter time setting
  - Press on DDMM display to enter date setting
  - In setting modes:
    - Button 1: Increment current field
    - Button 2: Switch to next field or save and exit

## IR Remote Controls

Support for Akai CD and LG Game Remote controls with various functions including mode selection, font changing, and brightness adjustment.

## Configuration Storage

Settings are stored in EEPROM and persist across power cycles. Use `AT&W` to save current settings and `AT&Y` to reload them.
