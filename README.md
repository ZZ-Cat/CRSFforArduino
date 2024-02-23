# CRSF for Arduino

[![GitHub release (latest by date)](https://img.shields.io/github/v/release/ZZ-Cat/CRSFforArduino)](https://github.com/ZZ-Cat/CRSFforArduino/releases/latest)
[![GitHub license](https://img.shields.io/github/license/ZZ-Cat/CRSFforArduino)](https://github.com/ZZ-Cat/CRSFforArduino/blob/Main-Trunk/LICENSE.md)
[![Conventional Commits](https://img.shields.io/badge/Conventional%20Commits-1.0.0-%23FE5196?logo=conventionalcommits&logoColor=white)](https://conventionalcommits.org)
[![Build Status](https://github.com/ZZ-Cat/CRSFforArduino/workflows/Arduino/badge.svg)](https://github.com/ZZ-Cat/CRSFforArduino/actions)
[![Build Status](https://github.com/ZZ-Cat/CRSFforArduino/workflows/PlatformIO/badge.svg)](https://github.com/ZZ-Cat/CRSFforArduino/actions)

## Written and developed by

Cassandra "ZZ Cat" Robinson

## Description

CRSF for Arduino brings the Crossfire Protocol to the Arduino ecosystem.  
This library enables you to connect either a TBS Crossfire or ExpressLRS receiver to your development board,
giving you access to telemetry and up to 16 11-bit proportional RC channels over a tried-and-true serial protocol.

The Crossfire Protocol (better known as CRSF) is used by both Team BlackSheep (in their Crossfire and Tracer receivers) and
ExpressLRS. The latter of the two are well-known in the FPV drone community for their ultra low latency and long range control
link.  
By pairing CRSF for Arduino with an ExpressLRS transmitter and receiver, you have a control link between your RC handset and your development project that is robust in tough RF environments.

## Prerequisites

### Arduino IDE

If you want to use CRSF for Arduino in the Arduino IDE, you need these:

- [Arduino IDE](https://www.arduino.cc/en/software)
  - ESP32 targets:
    - Additional Boards URL: `https://espressif.github.io/arduino-esp32/package_esp32_index.json`
    - [Arduino ESP32 Board Support Pack](https://github.com/espressif/arduino-esp32)
  - SAMD21 and SAMD51 targets:
    - Additional Boards URL: `https://adafruit.github.io/arduino-board-index/package_adafruit_index.json`
    - [Arduino SAMD Board Support Pack](https://github.com/arduino/ArduinoCore-samd)
    - [Adafruit SAMD Board Support Pack](https://github.com/adafruit/ArduinoCore-samd)
    - [Adafruit_ZeroDMA](https://github.com/adafruit/Adafruit_ZeroDMA)
  - Teensy 3.x and Teensy 4.x targets:
    - Additional Boards URL: `https://www.pjrc.com/teensy/package_teensy_index.json`
    - [Teensy Board Support Pack](https://github.com/PaulStoffregen/cores)

### Visual Studio Code and PlatformIO

If you want to use CRSF for Arduino with PlatoformIO and Visual Studio Code, you need these:

- [Visual Studio Code](https://code.visualstudio.com/) with the following extensions:
  - [C/C++ Extensions Pack](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools-extension-pack), which includes:
    - C/C++
    - C/C++ Themes
    - CMake
    - CMake Tools
  - [PlatformIO IDE](https://platformio.org/)

Pro tips:
In PlatformIO, CRSF for Arduino's dependencies are automatically installed, when you build your project for the first time.  
The `platformio.ini` file contains a list of development boards that is compatible with CRSF for Arduino.

## Installation

### Latest release

Currently, the only releases that are available right now are all pre-releases and are Major version 0.x.x.
This means that each pre-release is a snapshot of the current developmental state of CRSF for Arduino.
While every effort has gone into ensuring stability at the time that these releases are made, there may still be the odd gremlin or two that have stowed away somewhere and made it through my quality control.

With that being said, installation from the Releases tab is nearly identical to the installation steps above, save for how you acquire CRSF for Arduino, and that's by simply clicking on the `Source Code (zip)` link at the bottom of the release summary itself.

### PlatformIO dependency

Simply add `https://github.com/ZZ-Cat/CRSFforArduino.git @^ 1.0.0` to your `lib_deps` section in your `platformio.ini` file.  
PlatformIO will take care of the rest. This is fairly new, so any hiccups, don't hesitate to let me know via my Issues tab.

### Cloning the Main-Trunk (for contributors)

If you want bleeding edge features and want to help me out on developing CRSF for Arduino, this is how you go about it:

1. Click the green `<> code` button.
2. Click `download zip` and save it in a convenient location on your hard drive.
3. Extract the top level `CRSFforArduino` folder - For PlatformIO users, this is all you need to do. Arduino users, continue on from here.
4. Place the `CRSFforArduino` folder into your `libraries` directory.

Pro Tip:
If you want to easily switch between the Arduino IDE and PlatformIO, simply put the top level `CRSFforArduino` folder into your Arduino IDE's `libraries` directory as it is and leave everything there.  
Then, open up the _top level_ `CRSFforArduino` folder in Visual Studio Code. PlatformIO will automatically set you up and you should be good to go.

## Known issues and limitations

- CRSF for Arduino is not compatible with AVR based microcontrollers.
  - CRSF for Arduino uses a _lot_ of dynamic memory, to which all AVR microcontrollers simply don't have enough of.
  - There are ongoing tests to see what other reasons why AVR microcontrollers aren't compatible.
- DMA for both SAMD21 and SAMD51 targets is no longer available.
  - DMA may be re-factored later on down the track. However, it is not a priority right now.
- Software serial is not supported.
  - This is because software serial is not capable of running at the required baud rate of 420 KB/s.
  - This also means that CRSF for Arduino is restricted to using hardware serial only.
- CRSF for Arduino provides no sensor drivers for telemetry. This is by design.
  - There are already plenty of libraries out there that provide sensor drivers for your development board.
  - You are free to use any sensor library that you want with CRSF for Arduino.

## Software license

As always, I believe in freedom and I want to pass that freedom onto you.
Which is why I am proud to license CRSF for Arduino to you under the [GNU GPL v3](https://github.com/ZZ-Cat/CRSFforArduino/blob/Main-Trunk/LICENSE.md).

## Attributions

I give credit where credit is due. Because CRSF for Arduino isn't entirely my own idea, but built on the shoulders of giants. Here is a list of credits to those what helped to make this possible:

- Inspiration for this library
  - [ExpressLRS](https://github.com/ExpressLRS)
    - [Development Team](https://github.com/orgs/ExpressLRS/people)
    - [License](https://github.com/ExpressLRS/ExpressLRS/blob/master/LICENSE)
    - [Source Code](https://github.com/ExpressLRS/ExpressLRS)
    - [Website](https://www.expresslrs.org/3.0/)
  - [Team BlackSheep FPV](https://github.com/tbs-fpv) - The folks behind the CRSF protocol that both ExpressLRS and CRSF for Arduino uses.
- References for CRSF for Arduino
  - [BetaFlight](https://github.com/betaflight)
    - [Development Team](https://github.com/orgs/betaflight/people)
    - [License](https://github.com/betaflight/betaflight/blob/master/LICENSE)
    - [Source Code](https://github.com/betaflight/betaflight)
    - [Website](https://betaflight.com/)
  - [CRSF-WG](https://github.com/crsf-wg/crsf)
    - This is the official repository for The CRSF Protocol, which CRSF for Arduino is based on.
  - [RotorFlight](https://github.com/rotorflight)
    - [Development Team](https://github.com/rotorflight#credits)
    - [License](https://github.com/rotorflight/rotorflight-firmware/blob/master/LICENSE)
    - [Source Code](https://github.com/rotorflight/rotorflight-firmware)
- Third Party libraries
  - [Adafruit Industries](https://github.com/adafruit)
    - [Adafruit_ZeroDMA](https://github.com/adafruit/Adafruit_ZeroDMA)
      - [Author](https://github.com/PaintYourDragon)
      - [License](https://github.com/adafruit/Adafruit_ZeroDMA/blob/master/LICENSE)
    - [Website](https://www.adafruit.com/)

## Contributing

If you would like to contribute to the development to CRSF for Arduino, here is what you can do:

1. Read my [code of conduct](https://github.com/ZZ-Cat/CRSFforArduino/blob/Main-Trunk/CODE_OF_CONDUCT.md) and [contribution guidelines](https://github.com/ZZ-Cat/CRSFforArduino/blob/Main-Trunk/.github/CONTRIBUTING.md);
2. Fork CRSF for Arduino;
3. Make your contribution to the codebase;
4. Submit your changes in your fork as a Pull Request back to CRSF for Arduino.

Your contributions are very welcome, and if it benefits the project and community, it will be merged into the Main-Trunk.
