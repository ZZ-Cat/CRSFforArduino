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
