# CRSF for Arduino

[![GitHub release (latest by date)](https://img.shields.io/github/v/release/ZZ-Cat/CRSFforArduino)](https://github.com/ZZ-Cat/CRSFforArduino/releases/latest)
[![GitHub license](https://img.shields.io/github/license/ZZ-Cat/CRSFforArduino)](https://github.com/ZZ-Cat/CRSFforArduino/blob/Main-Trunk/LICENSE.md)
[![Conventional Commits](https://img.shields.io/badge/Conventional%20Commits-1.0.0-%23FE5196?logo=conventionalcommits&logoColor=white)](https://conventionalcommits.org)
[![Build Status](https://github.com/ZZ-Cat/CRSFforArduino/workflows/Arduino/badge.svg)](https://github.com/ZZ-Cat/CRSFforArduino/actions)
[![Build Status](https://github.com/ZZ-Cat/CRSFforArduino/workflows/PlatformIO/badge.svg)](https://github.com/ZZ-Cat/CRSFforArduino/actions)

## Written & developed by

Cassandra "ZZ Cat" Robinson

## Warnings

### This is not yet ready for prime time release

CRSF for Arduino is undergoing active development & is not yet ready for prime time release.
If you choose to use CRSF for Arduino in its current state, do so at your own risk.
Some features may be broken, bugged, untested, missing, or the code as a whole may resemble a pigeon flying by swinging its head around in circles.

Fear not! I am working on this library (aside from flying my helicopters & helping out with other heli-related projects) & I have every intention of making that stubborn pigeon fly by using its wings. No matter how much the basterd wants to insist on swinging its head around in circles to fly. =^/.~=

If you have spotted any bugs, something isn't working the way it should, or you have any suggestions on what you want to see in CRSF for Arduino, don't hesitate to open an Issue. The Discussions tab is also open, so if you want to chat to me about my library, feel free to do so there.

## Description

Traditional PWM RC receivers are becoming a thing of the past & (by extension) are getting harder & harder to come by.
CRSF is _the_ de-facto standard for RC nowadays, & it is time to bring it to the world of Arduino!
This means that your development board is now compatible with any receiver that runs ExpressLRS firmware.

## Why ExpressLRS?

For starters, it's an open source radio control link that offers incredibly low latency, long range (EG [100 kilometers on 2.4GHz](https://youtu.be/IjQYLyvai6s) without failsafe), & it's incredibly robust in rough RF environments.

An ExpressLRS receiver communicates to your development board through one of the board's Serial/UART ports.
This provides you with up to 16 10-bit full resolution channels & telemetry without taking up unnecessary amounts of pin real estate.

## Prerequisites - Arduino IDE

If you want to use CRSF for Arduino in the Arduino IDE, you need these:

- [Arduino IDE](https://www.arduino.cc/en/software)
- [Arduino SAMD Board Support Pack](https://github.com/arduino/ArduinoCore-samd)
- [Adafruit SAMD Board Support Pack](https://github.com/adafruit/ArduinoCore-samd)
- [Adafruit_ZeroDMA](https://github.com/adafruit/Adafruit_ZeroDMA)

## Prerequisites - Visual Studio Code & PlatformIO

If you want to use CRSF for Arduino with PlatoformIO & Visual Studio Code, you need these:

- [Visual Studio Code](https://code.visualstudio.com/) with the following extensions:
  - [C/C++ Extensions Pack](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools-extension-pack), which includes:
    - C/C++
    - C/C++ Themes
    - CMake
    - CMake Tools
  - [PlatformIO IDE](https://platformio.org/)

Pro tips:
Up in the Arduino IDE prerequisites, I mention a list of third-party libraries & board support packages. If you are using PlatformIO, that list does not necessarily apply here. CRSF for Arduino comes with a `platformio.ini` file that already defines the necessary library dependencies & board support profiles. PlatformIO will automatically download all of the necessary files to setup your build environment.

Keep in mind that in the `platformio.ini` file, under the `platformio` section, I have set the `default_envs` to `adafruit_metro_m4`. This is what I am using to develop CRSF for Arduino on.
To set your desired target development board, you do `shift+ctrl+p ► PlatformIO: Switch Project Environment` & choose your development board from there. Each board is prepended with `env:[YOUR_TARGET_DEV_BOARD]` & for the default Adafruit Metro M4 Express target, it looks like this `env:adafruit_metro_m4` from the dropdown menu.

## Installation - Straight from the Main-Trunk

If you want bleeding edge features & want to help me out on developing CRSF for Arduino, this is how you go about it:

1. Click the green `<> code` button.
2. Click `download zip` & save it in a convenient location on your hard drive.
3. Extract the top level `CRSFforArduino` folder - For PlatformIO users, this is all you need to do. Arduino users, continue on from here.
4. Place the `CRSFforArduino` folder into your `libraries` directory.

Pro Tip:
If you're like me & subscribe to the "Both! Both? Both. Both is good." (as quoted by Miguel & Tulio from The Road to El Dorado) philosophy, you can put the top level `CRSFforArduino` folder into your Arduino IDE's `libraries` directory as is. Leave everything there. If you also have Visual Studio Code _and_ PlatformIO, you can go ahead & open up the _top level_ `CRSFforArduino` folder in Visual Studio Code, & PlatformIO will automatically set you up & (by rights) you should be good to go.
This gives you the ability to load up CRSF for Arduino in both the Arduino IDE & Visual Studio Code at the same time.

## Installation - From the Releases Section

Currently, the only releases that are available right now are all pre-releases & are Major version 0.x.x.
This means that each pre-release is a snapshot of the current developmental state of CRSF for Arduino.
While every effort has gone into ensuring stability at the time that these releases are made, there may still be the odd gremlin or two that have stowed away somewhere & made it through my quality control.

With that being said, installation from the Releases tab is nearly identical to the installation steps above, save for how you acquire CRSF for Arduino, & that's by simply clicking on the `Source Code (zip)` link at the bottom of the release summary itself.

## How to use CRSF for Arduino

### The API

1. Add my library to your sketch with ```#include "CRSFforArduino.h"```
2. Underneath that, you need to declare ```CRSFforArduino crsf = CRSFforArduino(&Serial1)```
3. In your ```setup()```, do ```crsf.begin()``` to start communicating with your connected ExpressLRS receiver.
4. In your ```loop()```, you need to call ```crsf.update()```, as this polls your receiver for new RC channels data.
5. Do ```crsf.getChannel(n)``` to get the raw value from your desired RC channel. Here, ```n``` refers to your channel number from 1 to 16.

If you want to convert the raw channel value to microseconds, do ```crsf.rcToUs(n)```, where ```n``` is the raw RC value you want to convert. ```rcToUs()``` will return the converted value in microseconds.

The example below demonstrates what your code should look like, using the instructions above:

```c++
/* Arduino is included here for shits & giggles. */
#include "Arduino.h"

/* 1. Add Cassie Robinson's CRSFforArduino.h library. */
#include "CRSFforArduino.h"

/* 2. Declare a CRSFforArduino object.
You can call it literally anything you want, as long as you tell CRSF for Arduino what serial port your receiver is connected to. */
CRSFforArduino crsf = CRSFforArduino(&Serial1);

void setup()
{
    Serial.begin(115200);
    while (!Serial)
    {
        ;
    }

    /* 3. Start communicating with a connected ELRS receiver. */
    crsf.begin();

    /* Pro tip: Always show a little message in the Serial Monitor to let you know that your sketch is initialized successfully. */
    Serial.println("Channels Example");
    delay(1000);
    Serial.println("Ready");
    delay(1000);
}

void loop()
{
    /* 4. Update the main loop with new RC data. */
    if (crsf.update())
    {
        /* Pro tip: Read back your RC channels data inside an if() statement.
        
        5. Here, your RC channel values are converted to microseconds, & are sent to the Serial Monitor. */
        Serial.print("RC Channels <A: ");
        Serial.print(crsf.rcToUs(crsf.getChannel(1)));
        Serial.print(", E: ");
        Serial.print(crsf.rcToUs(crsf.getChannel(2)));
        Serial.print(", T: ");
        Serial.print(crsf.rcToUs(crsf.getChannel(3)));
        Serial.print(", R: ");
        Serial.print(crsf.rcToUs(crsf.getChannel(4)));
        Serial.print(", Aux1: ");
        Serial.print(crsf.rcToUs(crsf.getChannel(5)));
        Serial.print(", Aux2: ");
        Serial.print(crsf.rcToUs(crsf.getChannel(6)));
        Serial.print(", Aux3: ");
        Serial.print(crsf.rcToUs(crsf.getChannel(7)));
        Serial.print(", Aux4: ");
        Serial.print(crsf.rcToUs(crsf.getChannel(8)));
        Serial.println(">");
    }
}
```

### Example Sketches

In the ```examples``` folder, there is a sketch called ```channels.cpp``` that I used to test this library.
It contains instructions on how to set your hardware up & how to connect your receiver to your development board.
It also details binding procedures (if needed), & the channel ranges & channel order.
The example sketch also demonstrates how to read RC channels data from your connected ExpressLRS receiver.

You can build this example to see how CRSF for Arduino works.

### Flashing - PlatformIO (VS Code)

Flashing is a lot simpler with PlatformIO when compared to the Arduino IDE.

1. Build your sketch ► ```pio run``` in your CLI or ```ctrl+alt+b``` on your keyboard.
2. Flash your sketch ► ```pio run -t upload``` in your CLI or ```ctrl+alt+u``` on your keyboard.

### Flashing - Arduino IDE

1. Select your target development board ► ```Tools ► Board```
2. Select the Serial Port that your board is connected to ► ```Tools ► Port```
3. Verify your sketch ► ```Sketch ► Verify/Compile``` from the menu or ```ctrl+r``` on your keyboard.
4. Upload your sketch ► ```Sketch ► Upload``` from the menu or ```ctrl+u```on your keyboard.

### Viewing RC data

1. Open up the Serial Monitor.
   - PlatformIO: Click the Serial Monitor tab, configure the port, baud rate & click ```Start Monitoring```.
   - PuTTY: In the configuration, select the ```Serial line```, set the ```Connection type``` to ```Serial``` & set the ```Speed``` to your baud rate setting (default is 115200). Then, click ```Open```.
   - Arduino IDE: ```Tools ► Serial Monitor``` from the menu or ```ctrl+shift+m``` on your keyboard.
2. Your RC channel values will be there, & they will change as you move the sticks on your RC handset.

## Compatible development boards

CRSF for Arduino is designed to be compatible with modern hardware.
While CRSF for Arduino is primarily developed on the Adafruit Metro M4 Express, here is a list of target development boards CRSF for Arduino is compatible with (Keep in mind that this list is not exhaustive, & actual compatibility with everything listed here may be untested):

- SAMD21 based boards:
  - Adafruit Feather M0 & all of its variants, including the Adafruit Feather M0 Express
  - Adafruit ItsyBitsy M0 Express
  - Adafruit Metro M0 Express
  - Adafruit QtPy M0
  - Adafruit Trinket M0
  - Arduino MKR series:
    - MKR 1000
    - MKR Fox 1200
    - MKR GSM 1400
    - MKR NB 1500
    - MKR VIDOR 4000
    - MKR WAN 1300 & MKR WAN 1310
    - MKR WIFI 1010
    - MKR Zero
  - Arduino Nano 33 IoT
  - Arduino Zero
  - Seeed Studio XIAO SAMD21
- SAMD51 based boards:
  - Adafruit Feather M4 Express
  - Adafruit Grand Central M4
  - Adafruit ItsyBitsy M4 Express
  - Adafruit Metro M4 Express
  - Adafruit Metro M4 Express AirLift Lite
- SAME51 based boards:
  - Adafruit Feather M4 CAN Express

Compatibility with other microcontroller boards may be added in future, if there is demand for it. Keep in mind that this will be subject to hardware limitations of the host microcontroller itself.

Generally speaking, if the host microcontroller's UART peripheral supports 420k bauds (or higher), it is a likely candidate for CRSF for Arduino. Bonus points if the microcontroller has DMA (Direct Memory Access), because this helps immensely to capture RC data & send out telemetry data.

The only limitation so far is the microcontroller's ability to change the UART's data order. CRSF requires data to be transmitted & received in MSB (Most Significant Bit) First. Traditionally, UART is transmitted & received in LSB (Least Significant Bit) First. ATSAMD21 & ATSAMD51 microcontrollers are the only two Arduino-compatible microcontrollers (that I am aware of) that have the ability to select what data order the UART is sent & received as.

I am now aware that Arduino are making an R4 of their UNO & Adafruit are making a Metro M7. This is exciting news for me, because I would like to make these development boards fully compatible with CRSF for Arduino as & when their respective underlying code & toolchain support is added.

## AVR based microcontrollers are not compatible

Development boards of yesteryear that were built around ATmega AVR microcontrollers such as the 328, 32u4, 2560 & 4809 are incompatible with CRSF for Arduino.
Their processing capabilities (such as their limited 16 MHz CPU speed) simply are not enough to meet the requirements of the CRSF protocol.

The following is a list of known development boards that are incompatible.
This list is not exhaustive, but it should give you an idea of what _not_ to flash CRSF for Arduino onto. If you even try to do so, CRSF for Arduino will simply not function (instead of allegedly exhibiting ["strange behaviour"](https://github.com/ZZ-Cat/CRSFforArduino/issues/12)):

- Adafruit Metro 328
- Adafruit Feather 32u4 (including all of its variants)
- Arduino Diecimila
- Arduino Duemilanove
- Arduino Leonardo
- Arduino Mega2560
- Arduino Micro
- Arduino Nano
- Arduino Nano Every
- Arduino UNO R3
- Arduino UNO WiFi R2

There is better hardware out there that has the same form factor as these old development boards.
Here are some examples:

- These have an equivalent "breadboard-friendly" form factor that is similar to the Arduio Micro & the Arduino Nano:
  - Adafruit Feather M0 (all variants, including the Feather M0 Express)
  - Adafruit Feather M4 Express
  - Seeed Studio XIAO SAMD21
- These have the "Arduino R3" form factor - The same shape as the Arduino UNO:
  - Adafruit:
    - Metro M0 Express
    - Metro M4 AirLift Lite
    - Metro M4 Express
  - Arduino:
    - Zero
- These have the "Arduino Mega" form factor:
  - Adafruit:
    - Grand Central M4

Many of Arduino's [discontinued development boards](https://docs.arduino.cc/retired/) are also incompatible with CRSF for Arduino.

## Compatible receivers

Generally speaking, if your transmitter & receiver combo supports ExpressLRS or TBS Crossfire, it's automatically compatible.
Keep in mind that CRSF for Arduino is tested almost exclusively on ExpressLRS hardware.

For wiring, here is how you do it:

- GND (on the receiver; can also be printed on the receiver as a minus sign '-') can go to any GND pin on your development board.
- 5V (on the receiver; can also be printed on the receiver as 'vcc' or simply a plus sign '+') goes to the 5V pin of your development board. This power will come directly off of USB power or 5V stepped down from your development board's main power connector.
- T (or Tx) on your receiver goes to ►Rx/Pin0 on your development board. This step can catch you out, if you're not careful.
- R (or Rx) on your reciever goes to ◄Tx/Pin1 on your development board. This step can catch you out, if you're not careful.

**NB:** Some development boards (especially smaller ones) may not route the primary UART port to the default 0 & 1 pins, as these pins may be used for other purposes or they simply are not physically available to you on the device.
As a workaround, CRSF for Arduino automatically selects the closest available pins that has a suitable UART port.
You _must_ refer to your development board's documentation to determine where your default Tx & Rx pins are. These are the pins that CRSF for Arduino will use.

## Telemetry

Currently, there is no telemetry feedback from the host microcontroller's side... _yet!_
This is coming soon.

## Software license

As always, I believe in freedom & I want to pass that freedom onto you.
Which is why I am proud to license CRSF for Arduino to you under the [GNU GPL v3](https://github.com/ZZ-Cat/CRSFforArduino/blob/Main-Trunk/LICENSE.md).

## Attributions

I give credit where credit is due. Because CRSF for Arduino isn't entirely my own idea, but built on the shoulders of giants. Here is a list of credits to those what helped to make this possible:

- Inspiration for this library
  - [ExpressLRS](https://github.com/ExpressLRS)
    - [Development Team](https://github.com/orgs/ExpressLRS/people)
    - [License](https://github.com/ExpressLRS/ExpressLRS/blob/master/LICENSE)
    - [Source Code](https://github.com/ExpressLRS/ExpressLRS)
    - [Website](https://www.expresslrs.org/3.0/)
  - [Team BlackSheep FPV](https://github.com/tbs-fpv) - The folks behind the CRSF protocol that both ExpressLRS & CRSF for Arduino uses.
  - [This issue](https://github.com/tbs-fpv/freedomtx/issues/26) on [FreedomTX's repository.](https://github.com/tbs-fpv/freedomtx/issues/26)
    - This gets a mention here, because I will benefit _greatly_ from an officially documented public repository for the CRSF protocol. The evolution of the protocol itself will help shape CRSF for Arduino, & I will be able to refer to that in addition to my references here.
- References for CRSF for Arduino
  - [BetaFlight](https://github.com/betaflight)
    - [Development Team](https://github.com/orgs/betaflight/people)
    - [License](https://github.com/betaflight/betaflight/blob/master/LICENSE)
    - [Source Code](https://github.com/betaflight/betaflight)
    - [Website](https://betaflight.com/)
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
