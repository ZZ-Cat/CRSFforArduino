# CRSF for Arduino

[![GitHub release (latest by date)](https://img.shields.io/github/v/release/ZZ-Cat/CRSFforArduino)](https://github.com/ZZ-Cat/CRSFforArduino/releases/latest)
[![GitHub license](https://img.shields.io/github/license/ZZ-Cat/CRSFforArduino)](https://github.com/ZZ-Cat/CRSFforArduino/blob/Main-Trunk/LICENSE.md)  
![Quality Control](https://github.com/ZZ-Cat/CRSFforArduino/actions/workflows/quality_control.yml/badge.svg)  
[![Conventional Commits](https://img.shields.io/badge/Conventional%20Commits-1.0.0-%23FE5196?logo=conventionalcommits&logoColor=white)](https://conventionalcommits.org)

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

## Documentation

CRSF for Arduino's documentation is now live.  
Feel free to peruse the [Wiki](https://github.com/ZZ-Cat/CRSFforArduino/wiki) at your leisure.  
For new users, it is _strongly_ recommended that you read the documentation in its entirety.

## Features

CRSF for Arduino comes packaged with these features:

- 16 11-bit proportional RC channels.
- Diverse modern hardware support from the following architectures:
  - ESP32, RP2040, SAMD21, SAMD5x, SAME5x, STM32, and Teensy 4.x.
- Flight Modes, both custom and standard.
  - Standard Flight Modes are based on the Flight Modes from Betaflight 4.3.
  - Custom Flight Modes use text-based strings that you can assign custom names to for your bespoke purposes.
  - Both Standard and Custom Flight Modes can be sent as Telemetry back to your controller.
- Event-driven API.
- Fully configurable.
  - CRSF for Arduino can be tailored to suit the needs of your individual projects.
- Telemetry.
  - Attitude.
  - Barometric Altitude.
  - Battery.
  - Flight Modes.
  - GPS.

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
