# CRSF for Arduino

[![GitHub release (latest by date)](https://img.shields.io/github/v/release/ZZ-Cat/CRSFforArduino)](https://github.com/ZZ-Cat/CRSFforArduino/releases/latest)
[![GitHub license](https://img.shields.io/github/license/ZZ-Cat/CRSFforArduino)](https://github.com/ZZ-Cat/CRSFforArduino/blob/Main-Trunk/LICENSE.md)
[![Conventional Commits](https://img.shields.io/badge/Conventional%20Commits-1.0.0-%23FE5196?logo=conventionalcommits&logoColor=white)](https://conventionalcommits.org)
[![Build Status](https://github.com/ZZ-Cat/CRSFforArduino/workflows/Arduino/badge.svg)](https://github.com/ZZ-Cat/CRSFforArduino/actions)
[![Build Status](https://github.com/ZZ-Cat/CRSFforArduino/workflows/PlatformIO/badge.svg)](https://github.com/ZZ-Cat/CRSFforArduino/actions)

## Written and developed by

Cassandra "ZZ Cat" Robinson

## Warnings

### This is not yet ready for prime time release

CRSF for Arduino is undergoing active development and is not yet ready for prime time release.
If you choose to use CRSF for Arduino in its current state, do so at your own risk.
Some features may be broken, bugged, untested, missing, or the code as a whole may resemble a pigeon flying by swinging its head around in circles.

Fear not! I am working on this library (aside from flying my helicopters and helping out with other heli-related projects) and I have every intention of making that stubborn pigeon fly by using its wings. No matter how much the basterd wants to insist on swinging its head around in circles to fly. =^/.~=

If you have spotted any bugs, something isn't working the way it should, or you have any suggestions on what you want to see in CRSF for Arduino, don't hesitate to open an Issue. The Discussions tab is also open, so if you want to chat to me about my library, feel free to do so there.

## Description

CRSF for Arduino brings the Crossfire Protocol to the Arduino ecosystem.  
This library enables you to connect either a TBS Crossfire or ExpressLRS receiver to your development board,
giving you access to telemetry and up to 16 11-bit proportional RC channels over a tried-and-true serial protocol.

The Crossfire Protocol (better known as CRSF) is used by both Team BlackSheep (in their Crossfire and Tracer receivers) and
ExpressLRS. The latter of the two are well-known in the FPV drone community for their ultra low latency and long range control
link.  
By pairing CRSF for Arduino with an ExpressLRS transmitter and receiver, you have a control link between your RC handset and your development project that is robust in tough RF environments.

## Prerequisites - Arduino IDE

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

## Prerequisites - Visual Studio Code and PlatformIO

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

## Installation - Straight from the Main-Trunk

If you want bleeding edge features and want to help me out on developing CRSF for Arduino, this is how you go about it:

1. Click the green `<> code` button.
2. Click `download zip` and save it in a convenient location on your hard drive.
3. Extract the top level `CRSFforArduino` folder - For PlatformIO users, this is all you need to do. Arduino users, continue on from here.
4. Place the `CRSFforArduino` folder into your `libraries` directory.

Pro Tip:
If you want to easily switch between the Arduino IDE and PlatformIO, simply put the top level `CRSFforArduino` folder into your Arduino IDE's `libraries` directory as it is and leave everything there.  
Then, open up the _top level_ `CRSFforArduino` folder in Visual Studio Code. PlatformIO will automatically set you up and you should be good to go.

## Installation - From the Releases Section

Currently, the only releases that are available right now are all pre-releases and are Major version 0.x.x.
This means that each pre-release is a snapshot of the current developmental state of CRSF for Arduino.
While every effort has gone into ensuring stability at the time that these releases are made, there may still be the odd gremlin or two that have stowed away somewhere and made it through my quality control.

With that being said, installation from the Releases tab is nearly identical to the installation steps above, save for how you acquire CRSF for Arduino, and that's by simply clicking on the `Source Code (zip)` link at the bottom of the release summary itself.

## How to use CRSF for Arduino

### The API

1. Add my library to your sketch with `#include "CRSFforArduino.h"`
2. Underneath that, you need to declare `CRSFforArduino crsf = CRSFforArduino()`. For now, you can leave the parentheses empty. There is ongoing work to allow you to specify what pins your receiver is connected to. For now, it defaults to pins 0 and 1 for Tx and Rx respectively.
3. In your `setup()`, do `crsf.begin()` to start communicating with your connected ExpressLRS receiver. In case something goes wrong, `crsf.begin()` returns a boolean value of `true` if initialisation is successful, and `false` if it is not.
4. In your `loop()`, you need to call `crsf.update()`. This handles all of the data processing (including receiving RC channels and sending telemetry) and should be called as often as possible. You no longer need to read back the return value of `crsf.update()`, as it no longer returns anything. Everything is handled internally now.
5. To read your RC channel values, use `crsf.readRcChannel(n)`. Here, `n` refers to your channel number from 1 to 16.

If you want to read the raw RC value instead of microseconds from your RC channel, `readRcChannel()` can take an optional second argument of `true` to return the raw RC value instead of microseconds. For example, `crsf.readRcChannel(1, true)` will return the raw RC value from channel 1.

The example below demonstrates what your code should look like, using the instructions above:

```c++
/* Arduino is included here for shits and giggles. */
#include "Arduino.h"

/* 1. Add Cassie Robinson's CRSFforArduino.h library. */
#include "CRSFforArduino.h"

/* 2. Declare a CRSFforArduino object.
You can call it literally anything you want. */
CRSFforArduino crsf = CRSFforArduino();

void setup()
{
    Serial.begin(115200);
    while (!Serial)
    {
        ;
    }
    Serial.println("Channels Example");

    /* 3. Start communicating with a connected ELRS receiver. */
    if (!crsf.begin())
    {
        Serial.println("CRSF for Arduino initialization failed!");
        while (1)
        {
            ;
        }
    }

    /* Pro tip: Always show a little message in the Serial Monitor to let you know that your sketch is initialized successfully. */
    Serial.println("Ready");
    delay(1000);
}

void loop()
{
    /* 4. Update the main loop with new RC data. */
    crsf.update();

    /* 5. Here, you can do whatever you want with your RC channel values.
    In this example, your RC channel values are printed to the Serial Monitor every 100 milliseconds. */
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint >= 100)
    {
        lastPrint = millis();

        Serial.print("RC Channels <A: ");
        Serial.print(crsf.readRcChannel(1));
        Serial.print(", E: ");
        Serial.print(crsf.readRcChannel(2));
        Serial.print(", T: ");
        Serial.print(crsf.readRcChannel(3));
        Serial.print(", R: ");
        Serial.print(crsf.readRcChannel(4));
        Serial.print(", Aux1: ");
        Serial.print(crsf.readRcChannel(5));
        Serial.print(", Aux2: ");
        Serial.print(crsf.readRcChannel(6));
        Serial.print(", Aux3: ");
        Serial.print(crsf.readRcChannel(7));
        Serial.print(", Aux4: ");
        Serial.print(crsf.readRcChannel(8));
        Serial.println(">");
    }
}
```

If you want to transmit data from your GPS module as telemetry, do `crsf.telemetryWriteGPS(lat, lon, alt, spd, gCourse, numSats)`, where `lat` and `lon` are your GPS' location data in decimal degrees, `alt` is your GPS' height above sea level in centimetres (cm), `gCourse` is your GPS' course over ground (AKA "Compass/heading") in degrees, and `numSats` is your GPS' number of satellites that it is seeing.
You can use this function in the same context as your calling code that polls your GPS module:

```c++
#include "Arduino.h"
#include "wiring_private.h"

/* 1. Add Cassie Robinson's CRSF for Arduino library. */
#include "CRSFforArduino.h"

/* 2. Add Mikal Hart's TinyGPS++ library.
Source: https://tinyurl.com/4v4y5s3t */
#include "TinyGPSPlus.h"

// Create a Uart object for your GPS module.
// For more information, see: https://tinyurl.com/2p9brewf
Uart gpsSerial(&sercom1, 11, 10, SERCOM_RX_PAD_0, UART_TX_PAD_2);

// Create a TinyGPSPlus object.
TinyGPSPlus gps;

// GPS data variables.
float lat, lon, alt, spd, gCourse;
int numSats;

/* 3. Declare a CRSFforArduino object. */
CRSFforArduino crsf = CRSFforArduino();

void setup()
{
    Serial.begin(115200);
    while (!Serial)
    {
        ;
    }

    /* 4. Initialize your GPS module. */
    gpsSerial.begin(9600);

    // We need to set pins 10 and 11 to be SERCOM pins.
    pinPeripheral(10, PIO_SERCOM);
    pinPeripheral(11, PIO_SERCOM);

    /* 5. Initialize CRSF for Arduino. */
    crsf.begin();

    /* Pro tip: Always show a little message in the Serial Monitor to let you know that your sketch is initialized successfully. */
    Serial.println("GPS Telemetry Example");
    delay(1000);
    Serial.println("Ready");
    delay(1000);
}

void loop()
{
    /* 6. Poll your GPS module for new data. */
    while (gpsSerial.available() > 0)
    {
        if (gps.encode(gpsSerial.read()))
        {
            /* 7. Get the GPS data. */
            lat = gps.location.lat();
            lon = gps.location.lng();
            alt = gps.altitude.meters();
            spd = gps.speed.kmph();
            gCourse = gps.course.deg();
            numSats = gps.satellites.value();

            /* 8. Send the GPS data as telemetry. */
            crsf.telemetryWriteGPS(lat, lon, alt, spd, gCourse, numSats);
        }
    }

    /* 9. Update the main loop with new RC data. */
    crsf.update();

    /* 10. Here, you can do whatever you want with your RC channel values.
    In this example, your RC channel values are printed to the Serial Monitor every 100 milliseconds. */
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint >= 100)
    {
        lastPrint = millis();

        Serial.print("RC Channels <A: ");
        Serial.print(crsf.readRcChannel(1));
        Serial.print(", E: ");
        Serial.print(crsf.readRcChannel(2));
        Serial.print(", T: ");
        Serial.print(crsf.readRcChannel(3));
        Serial.print(", R: ");
        Serial.print(crsf.readRcChannel(4));
        Serial.print(", Aux1: ");
        Serial.print(crsf.readRcChannel(5));
        Serial.print(", Aux2: ");
        Serial.print(crsf.readRcChannel(6));
        Serial.print(", Aux3: ");
        Serial.print(crsf.readRcChannel(7));
        Serial.print(", Aux4: ");
        Serial.print(crsf.readRcChannel(8));
        Serial.println(">");
    }
}

// SAMD51 SERCOM1 IRQ handlers.
void SERCOM1_0_Handler()
{
    gpsSerial.IrqHandler();
}

void SERCOM1_1_Handler()
{
    gpsSerial.IrqHandler();
}

void SERCOM1_2_Handler()
{
    gpsSerial.IrqHandler();
}

void SERCOM1_3_Handler()
{
    gpsSerial.IrqHandler();
}
```

### Example Sketches

In the `examples` folder, there are three sketches that I use to test this library:

- `channels.ino`
  This example demonstrates how to read RC channels data from your connected receiver.
- `flight_modes.ino`
  This example demonstrates how one may implement Flight Modes in their project and use Flight Modes Telemetry.
- `telemetry.ino`
  This example demonstrates how to pass data from your sensors to your controller using CRSF for Arduino.

You can build these examples to see how CRSF for Arduino works.

### Configuration

`CFA_Config.hpp` is used to tailor CRSF for Arduino for your project's needs.
For more information, please view #47.

### Flashing - PlatformIO (VS Code)

Flashing is a lot simpler with PlatformIO when compared to the Arduino IDE.

1. Build your sketch ► `pio run -e <board_name>` in your CLI.
2. Flash your sketch ► `pio run -e <board_name> -t upload` in your CLI.

Replace `<board_name>` with your chosen development board in the `platformio.ini` configuration file.
For example, if you are using an Adafruit Metro M4 Express, you would use `pio run -e adafruit_metro_m4` to build your sketch and `pio run -e adafruit_metro_m4 -t upload` to flash your sketch to your Metro M4 Express.

### Flashing - Arduino IDE

1. Select your target development board ► `Tools ► Board`
2. Select the Serial Port that your board is connected to ► `Tools ► Port`
3. Verify your sketch ► `Sketch ► Verify/Compile` from the menu or `ctrl+r` on your keyboard.
4. Upload your sketch ► `Sketch ► Upload` from the menu or `ctrl+u`on your keyboard.

### Viewing RC data

1. Open up the Serial Monitor.
   - PlatformIO: Click the Serial Monitor tab, configure the port, baud rate and click `Start Monitoring`.
   - PuTTY: In the configuration, select the `Serial line`, set the `Connection type` to `Serial` and set the `Speed` to your baud rate setting (default is 115200). Then, click `Open`.
   - Arduino IDE: `Tools ► Serial Monitor` from the menu or `ctrl+shift+m` on your keyboard.
2. Your RC channel values will be there, and they will change as you move the sticks on your RC handset.

In the Arduino IDE, if you prefer to use the Serial Plotter, set `USE_SERIAL_PLOTTER` to 1 in `telemetry.ino` before flashing it to your development board.  
Please note: Support for the Serial Plotter is experimental at this stage.

### Viewing telemetry

1. Flash the telemetry example to your target development board.
2. Connect your receiver to the default Rx and Tx pins on your development board.
3. In your RC handset go to `Model Settings ► Telemetry`.
4. Click `Discover new` and your telemetry should be automatically populated on your screen.

## Compatible development boards

CRSF for Arduino is compatible with these development boards:

- ESP32 based boards:
  - Adafruit Feather ESP32
  - Adafruit Feather ESP32-S2
  - Adafruit Feather ESP32-S3 (2 MB PSRAM)
  - Adafruit Feather ESP32-S3 (NO PSRAM)
  - Adafruit ItsyBitsy ESP32
  - Adafruit Metro ESP32-S2
  - Adafruit QtPy ESP32
  - Adafruit QtPy ESP32 Pico
  - Adafruit QtPy ESP32-C3
  - Adafruit QtPy ESP32-S2
  - Adafruit QtPy ESP32-S3
  - Arduino Nano ESP32
  - Espressif ESP32-C3-DevKit
  - Espressif ESP32-S3-DevKit
  - Seeed Studio XIAO ESP32-C3
  - Seeed Studio XIAO ESP32-S3
  - SparkFun ESP32 RedBoard IoT
  - SparkFun ESP32 Thing
  - SparkFun ESP32 Thing Plus
  - SparkFun ESP32-S2 Thing Plus
- SAMD21 based boards:
  - Adafruit Feather M0 and all of its variants, including the Adafruit Feather M0 Express
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
    - MKR WAN 1300 and MKR WAN 1310
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
- Teensy 3.x  
  **NB:** The entire Teensy 3.x line is discontinued by the manufacturer, and is _not_ recommended for new projects.
- Teensy 4.x

In order for CRSF for Arduino to run, the host microcontroller _must_ meet these minimum requirements:

- Core Clock Speed: 48 MHz or higher.
- CPU: ARM Cortex M0+ or later.
- UART Baud Rate: 420 KB/s.

If your development board is not on the list above and it meets the minimum requirements, consider opening up an [Issue](https://github.com/ZZ-Cat/CRSFforArduino/issues/new/choose) and I will work with you to add support for your development board.  
You need to be available for testing functionality to ensure CRSF for Arduino is working properly, as well as helping me debug any issues that crop up.

## Compatible receivers

Any receiver that uses the Crossfire Protocol is compatible with CRSF for Arduino.  
Generally this applies to any TBS Crossfire and TBS Tracer receiver, and any receiver that's running ExpressLRS firmware.

For wiring, here is how you do it:

- GND (on the receiver; can also be printed on the receiver as a minus sign '-') can go to any GND pin on your development board.
- 5V (on the receiver; can also be printed on the receiver as 'vcc' or simply a plus sign '+') goes to the 5V pin of your development board. This power will come directly off of USB power or 5V stepped down from your development board's main power connector.
- T (or Tx) on your receiver goes to ►Rx/Pin0 on your development board. This step can catch you out, if you're not careful.
- R (or Rx) on your reciever goes to ◄Tx/Pin1 on your development board. This step can catch you out, if you're not careful.

**NB:** Some development boards (especially smaller ones) may not route the primary UART port to the default 0 and 1 pins, as these pins may be used for other purposes or they simply are not physically available to you on the device.
As a workaround, CRSF for Arduino automatically selects the closest available pins that has a suitable UART port.
You _must_ refer to your development board's documentation to determine where your default Tx and Rx pins are. These are the pins that CRSF for Arduino will use.

**Regarding logic levels:**  
The Rx and Tx lines on your receiver is 3.3 volts. _NOT_ 5 volts.  
If your development board outputs a logic level of 5 volts on its serial pins, you _must_ use a level shifter. Otherwise you will kill the Rx and Tx pins on your receiver.

## Telemetry

The following telemetry data is supported:

- Attitude/Artificial Horizon data:
  - Roll, Pitch and Yaw are all in decidegrees
- Barometric Altitude and Variometer data:
  - Altitude (in decimetres)
  - Variometer (in centmetres per second)
- Battery sensor data:
  - Average battery cell voltage (in millivolts)
  - Current (in milliamperes)
  - Fuel (in milliampere-hours)
  - Percent remaining
- Flight Modes:
  - Betaflight's Flight Modes:
    - Acro
    - Angle
    - Disarmed (is registered with a `*` flag)
    - Failsafe
    - GPS Rescue (Including Wait for GPS Fix)
    - Horizon
    - Passthrough
    - "Air mode"
- GPS data:
  - Latitude (in decimal degrees)
  - Longitude (in decimal degrees)
  - Altitude (in centimetres)
  - Speed (in centimetres per second)
  - Course over ground (in degrees)
  - Number of satellites

## Known issues and limitations

- CRSF for Arduino is not compatible with AVR based microcontrollers.
  - This is because the AVR microcontrollers are simply not powerful enough to meet the minimum requirements of the CRSF protocol.
- DMA for both SAMD21 and SAMD51 targets is currently broken, and is disabled for the time being.
  If you build CRSF for Arduino with `USE_DMA` enabled, you will see this warning message: `DMA is enabled. This is an experimental feature and may not work as expected.`
  At this point, DMA may be removed from CRSF for Arduino, as it brings _very little_ benefit to the project... if any at all.
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
  - [This issue](https://github.com/tbs-fpv/freedomtx/issues/26) on [FreedomTX's repository.](https://github.com/tbs-fpv/freedomtx/issues/26)
    - This gets a mention here, because I will benefit _greatly_ from an officially documented public repository for the CRSF protocol. The evolution of the protocol itself will help shape CRSF for Arduino, and I will be able to refer to that in addition to my references here.
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

## Contributing

If you would like to contribute to the development to CRSF for Arduino, here is what you can do:

1. Read my [code of conduct](https://github.com/ZZ-Cat/CRSFforArduino/blob/Main-Trunk/CODE_OF_CONDUCT.md) and [contribution guidelines](https://github.com/ZZ-Cat/CRSFforArduino/blob/Main-Trunk/.github/CONTRIBUTING.md);
2. Fork CRSF for Arduino;
3. Make your contribution to the codebase;
4. Submit your changes in your fork as a Pull Request back to CRSF for Arduino.

Your contributions are very welcome, and if it benefits the project and community, it will be merged into the Main-Trunk.
