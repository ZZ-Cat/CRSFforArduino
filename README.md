# CRSF for Arduino

A library for communicating with ExpressLRS receivers.

## Written & developed by

Cassandra "ZZ Cat" Robinson

## Warning

CRSFforArduino is undergoing active development & is not yet ready for prime time release.
If you choose to use CRSFforArduino in its current state, do so at your own risk.
Some features may be broken, bugged, untested, missing, or the code as a whole may resemble a pigeon flying by swinging its head around in circles.

Fear not! I am working on this library (aside from flying my helicopters & helping out with other heli-related projects) & I have every intention of making that stubborn pigeon fly by using its wings. No matter how much the basterd wants to insist on swinging its head around in circles to fly. =^/.~=

If you have spotted any bugs, something isn't working the way it should, or you have any suggestions on what you want to see in CRSFforArduino, don't hesitate to open an Issue. The Discussions tab is also open, so if you want to chat to me about my library, feel free to do so there.

## Description

This library allows your target development board to communicate with ExpressLRS receivers, giving you the ability to use modern RC handsets with your DIY robotics & remotely operated vehicle projects.

## Why ELRS?

For starters, it's an open source radio control link that offers incredibly low latency, long range (EG close to 40km at 100mW of transmit power on 2.4GHz without failsafe), & it's incredibly robust in rough RF environments.

An ELRS receiver communicates to a connected host microcontroller through UART using a protocol known as CRSF (short for Crossfire). This is more robust than traditional PWM receivers, & only two pins are used for bidirectional serial communication.

Also, I wanted to adapt ExpressLRS to the Arduino ecosystem to prove to myself that it could be done.

## Installation

### Prerequisites

You need these before you can use CRSFforArduino.

- [Adafruit's SAMD Board Support package](https://github.com/adafruit/ArduinoCore-samd)
- [Adafruit's Zero DMA driver](https://github.com/adafruit/Adafruit_ZeroDMA)
- [Arduino's SAMD Board Support package](https://github.com/arduino/ArduinoCore-samd)
- [Arduino IDE](https://www.arduino.cc/en/software) If you are using the VSCode plugin, use the Arduino IDE version 1.8.19. Otherwise, use the very latest version of the Arduino IDE.

### Download CRSFforArduino

1. Click the green code button & hit "Download ZIP".
2. Extract that to your libraries' directory. EG ```C:\Users\..\Documents\Arduino\libraries```
3. Start up the Arduino IDE (or VSCode, if you're using that instead).

## How to use CRSFforArduino

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
You can call it literally anything you want, as long as you tell CRSFforArduino what serial port your receiver is connected to. */
CRSFforArduino crsf = CRSFforArduino(&Serial1)

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

In the ```examples``` folder, there is a sketch called ```channels.ino``` that I used to test this library.
It contains instructions on how to set your hardware up & how to connect your receiver to your development board.
It also details binding procedures (if needed), & the channel ranges & channel order.
The example sketch also demonstrates how to read RC channels data from your connected ExpressLRS receiver.

### Flashing

1. Verify your sketch.
2. Select your com port that your development board is connected to.
3. Upload your sketch to your development board.

### Viewing RC data

1. Open up the Serial Monitor.
2. Your RC channel values will be there, & they will change as you move the sticks on your RC handset.

## Compatible development boards

Here is a list of target development boards CRSFforArduino is compatible with:

- SAMD21 based boards:
  - Adafruit Crickit M0
  - Adafruit Feather M0 & variants
  - Adafruit Gemma M0
  - Adafruit ItsyBitsy M0 Express
  - Adafruit QtPy M0
  - Adafruit Trinket M0
  - Arduino MKR series
  - Arduino Zero
- SAMD51 based boards:
  - Adafruit Feather M4 Express
  - Adafruit Feather M4 CAN Express
  - Adafruit ItsyBitsy M4 Express
  - Adafruit Metro M4 Express
  - Adafruit Metro M4 Express AirLift Lite
- SAME51 based boards:
  - Adafruit Grand Central M4

The current version of this library is centered around the Microchip SAM D51 & SAM E51 microcontrollers. Therefore, this library will work with any development board (with the Arduino Rev3 form factor) that has either a SAM D51 or SAM E51 microcontroller.

Compatibility with other microcontroller boards may be added in the future, if there is demand for it, subject to hardware limitations of the host microcontroller itself.

Generally speaking, if the host microcontroller's UART peripheral support 400k bauds (or higher), it is a likely candidate for this library to support it.
If the host microcontroller also has DMA, this is an added bonus. DMA is no longer required (but it is still the preferred method).

## AVR based microcontrollers are not compatible

This includes all legacy ATmega microcontrollers & the development boards that were built around them.
Simply put, their capabilities are not there; & I will not be adding compatibility support for any of them.

## Compatible receivers

Generally speaking, if your receiver supports ExpressLRS, it's automatically compatible.

## Software license

As always, I believe in freedom & I want to pass that freedom onto you.
Which is why I am proud to license CRSFforArduino to you under the GNU GPL v3.

## Attributions

I give credit where credit is due. Because CRSFforArduino isn't entirely my own idea, but built on the shoulders of giants. Here is a list of credits to those what helped to make this possible:

- Inspiration for this library
  - [ExpressLRS](https://github.com/ExpressLRS)
    - [Development Team](https://github.com/orgs/ExpressLRS/people)
    - [License](https://github.com/ExpressLRS/ExpressLRS/blob/master/LICENSE)
    - [Source Code](https://github.com/ExpressLRS/ExpressLRS)
    - [Website](https://www.expresslrs.org/3.0/)
- References for CRSF implementation
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
