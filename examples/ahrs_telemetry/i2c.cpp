/**
 * @file i2c.h
 * @author Cassandra "ZZ Cat" Robinson (nicad.heli.flier@gmail.com)
 * @brief A hardware I2C driver for SAMD51, based on the TWI/I2C library for Arduino Zero.
 * @version 0.2.0
 * @date 2023-02-08
 *
 * @copyright Copyright (c) 2023, Cassandra "ZZ Cat" Robinson. All rights reserved.
 *
 */

extern "C" {
#include <string.h>
}

#include <Arduino.h>
#include <wiring_private.h>

#ifdef USE_TINYUSB
// For Serial when selecting TinyUSB
#include <Adafruit_TinyUSB.h>
#endif

#include "i2c.h"

/* I am not a fan of the sercom library using references to "Master" & "Slave" modes.
 * In 2021, there was a shift away from the "Master" & "Slave" terminology to "Controller" & "Target" as it is more inclusive.
 * Later on, I will create my own sercom library that uses the "Controller" & "Target" terminology.
 * For now, I am using the sercom library as-is - but I will be changing the terminology in the comments.
 * --Cassie Robinson, 2023-02-08
 */

 /**
  * @brief Construct a new I2C object
  *
  * @param s A pointer to the SERCOM object to use for I2C.
  * @param pinSDA The pin to use for I2C SDA.
  * @param pinSCL The pin to use for I2C SCL.
  */
I2C::I2C(SERCOM* s, uint8_t pinSDA, uint8_t pinSCL)
{
    this->sercom = s;
    this->_uc_pinSDA = pinSDA;
    this->_uc_pinSCL = pinSCL;
    transmissionBegun = false;
}

/**
 * @brief Start I2C as a controller.
 *
 * @param baudrate The baudrate to use for I2C. Defaults to 100000.
 */
void I2C::begin(uint32_t baudrate) {
    Serial.println("I2C: I2C::begin()");

    // Controller Mode
    Serial.println("I2C: I2C::begin() - sercom->initMasterWIRE()");
    sercom->initMasterWIRE(baudrate);
    sercom->enableWIRE();
    Serial.println("I2C: I2C::begin() - sercom->enableWIRE()");

    // Force PIO_SERCOM_ALT for now.
    pinPeripheral(_uc_pinSDA, PIO_SERCOM_ALT);
    pinPeripheral(_uc_pinSCL, PIO_SERCOM_ALT);

    Serial.println("I2C: I2C::begin() - done");
}

/**
 * @brief Start I2C as a target.
 *
 * @param address The address to use for I2C.
 * @param enableGeneralCall Whether or not to enable general call.
 */
void I2C::begin(uint8_t address, bool enableGeneralCall) {
    // Target mode
    sercom->initSlaveWIRE(address, enableGeneralCall);
    sercom->enableWIRE();

    // Force PIO_SERCOM_ALT for now.
    pinPeripheral(_uc_pinSDA, PIO_SERCOM_ALT);
    pinPeripheral(_uc_pinSCL, PIO_SERCOM_ALT);
}

/**
 * @brief Set the baud rate (clock speed) for I2C.
 *
 * @param baudrate The baudrate to use for I2C.
 */
void I2C::setClock(uint32_t baudrate) {

    // Exit if the baudrate is out of range
    if (baudrate < I2C_BAUDRATE_STANDARD || baudrate > I2C_BAUDRATE_FASTPLUS)
    {
        return;
    }

    sercom->disableWIRE();
    sercom->initMasterWIRE(baudrate);
    sercom->enableWIRE();
}

/**
 * @brief Stop I2C.
 *
 */
void I2C::end() {
    sercom->disableWIRE();
    sercom->resetWIRE();

    pinPeripheral(_uc_pinSDA, PIO_DIGITAL);
    pinPeripheral(_uc_pinSCL, PIO_DIGITAL);
}

/**
 * @brief Request a number of bytes from a target.
 *
 * @param address The address of the target to request bytes from.
 * @param quantity The number of bytes to request.
 * @param stopBit Whether or not to send a stop bit after the request.
 * @return uint8_t The number of bytes read.
 */
uint8_t I2C::requestFrom(uint8_t address, size_t quantity, bool stopBit)
{
    if (quantity == 0)
    {
        return 0;
    }

    size_t byteRead = 0;

    rxBuffer.clear();

    if (sercom->startTransmissionWIRE(address, WIRE_READ_FLAG))
    {
        // Read first data
        rxBuffer.store_char(sercom->readDataWIRE());

        // Connected to target
        for (byteRead = 1; byteRead < quantity; ++byteRead)
        {
            sercom->prepareAckBitWIRE();                          // Prepare Acknowledge
            sercom->prepareCommandBitsWire(WIRE_MASTER_ACT_READ); // Prepare the ACK command for the target
            rxBuffer.store_char(sercom->readDataWIRE());          // Read data and send the ACK
        }
        sercom->prepareNackBitWIRE();                           // Prepare NACK to stop target transmission
        //sercom->readDataWIRE();                               // Clear data register to send NACK

        if (stopBit)
        {
            sercom->prepareCommandBitsWire(WIRE_MASTER_ACT_STOP);   // Send Stop
        }
    }

    return byteRead;
}

uint8_t I2C::requestFrom(uint8_t address, size_t quantity)
{
    return requestFrom(address, quantity, true);
}

void I2C::beginTransmission(uint8_t address) {
    Serial.println("I2C: I2C::beginTransmission()");

    // save address of target and clear buffer
    Serial.println("I2C: I2C::beginTransmission() - save address of target and clear buffer");
    txAddress = address;
    txBuffer.clear();

    transmissionBegun = true;
    Serial.println("I2C: I2C::beginTransmission() - done");
}

// Errors:
//  0 : Success
//  1 : Data too long
//  2 : NACK on transmit of address
//  3 : NACK on transmit of data
//  4 : Other error
uint8_t I2C::endTransmission(bool stopBit)
{
    Serial.println("I2C: I2C::endTransmission()");
    transmissionBegun = false;

    // Start I2C transmission
    Serial.println("I2C: I2C::endTransmission() - Start I2C transmission");
    if (!sercom->startTransmissionWIRE(txAddress, WIRE_WRITE_FLAG))
    {
        Serial.println("I2C: I2C::endTransmission() - Start I2C transmission - Send stop bit");
        sercom->prepareCommandBitsWire(WIRE_MASTER_ACT_STOP);
        Serial.println("I2C: I2C::endTransmission() - Start I2C transmission - Address error");
        return 2;  // Address error
    }
    Serial.println("I2C: I2C::endTransmission() - Start I2C transmission - done");

    // Send all buffer
    Serial.println("I2C: I2C::endTransmission() - Send all buffer");
    while (txBuffer.available())
    {
        // Trying to send data
        if (!sercom->sendDataMasterWIRE(txBuffer.read_char()))
        {
            Serial.println("I2C: I2C::endTransmission() - Send all buffer - Send stop bit");
            sercom->prepareCommandBitsWire(WIRE_MASTER_ACT_STOP);
            Serial.println("I2C: I2C::endTransmission() - Send all buffer - Data error");
            return 3;  // Nack or error
        }
    }
    Serial.println("I2C: I2C::endTransmission() - Send all buffer - done");

    if (stopBit)
    {
        Serial.println("I2C: I2C::endTransmission() - Stop bit - Send stop bit");
        sercom->prepareCommandBitsWire(WIRE_MASTER_ACT_STOP);
        Serial.println("I2C: I2C::endTransmission() - Stop bit - done");
    }

    Serial.println("I2C: I2C::endTransmission() - done");
    return 0;
}

uint8_t I2C::endTransmission()
{
    return endTransmission(true);
}

size_t I2C::write(uint8_t ucData)
{
    // No writing, without begun transmission or a full buffer
    if (!transmissionBegun || txBuffer.isFull())
    {
        return 0;
    }

    txBuffer.store_char(ucData);

    return 1;
}

size_t I2C::write(const uint8_t* data, size_t quantity)
{
    // Try to store all data
    for (size_t i = 0; i < quantity; ++i)
    {
        // Return the number of data stored, when the buffer is full (if write return 0)
        if (!write(data[i]))
            return i;
    }

    // All data stored
    return quantity;
}

int I2C::available(void)
{
    return rxBuffer.available();
}

int I2C::read(void)
{
    return rxBuffer.read_char();
}

int I2C::peek(void)
{
    return rxBuffer.peek();
}

void I2C::flush(void)
{
    // Do nothing, use endTransmission(..) to force
    // data transfer.
}

void I2C::onReceive(void(*function)(int))
{
    onReceiveCallback = function;
}

void I2C::onRequest(void(*function)(void))
{
    onRequestCallback = function;
}

void I2C::onService(void)
{
    if (sercom->isSlaveWIRE())
    {
        if (sercom->isStopDetectedWIRE() ||
            (sercom->isAddressMatch() && sercom->isRestartDetectedWIRE() && !sercom->isMasterReadOperationWIRE())) //Stop or Restart detected
        {
            sercom->prepareAckBitWIRE();
            sercom->prepareCommandBitsWire(0x03);

            //Calling onReceiveCallback, if exists
            if (onReceiveCallback)
            {
                onReceiveCallback(available());
            }

            rxBuffer.clear();
        }
        else if (sercom->isAddressMatch())  //Address Match
        {
            sercom->prepareAckBitWIRE();
            sercom->prepareCommandBitsWire(0x03);

            if (sercom->isMasterReadOperationWIRE()) //Is a request ?
            {
                txBuffer.clear();

                transmissionBegun = true;

                //Calling onRequestCallback, if exists
                if (onRequestCallback)
                {
                    onRequestCallback();
                }
            }
        }
        else if (sercom->isDataReadyWIRE())
        {
            if (sercom->isMasterReadOperationWIRE())
            {
                uint8_t c = 0xff;

                if (txBuffer.available()) {
                    c = txBuffer.read_char();
                }

                transmissionBegun = sercom->sendDataSlaveWIRE(c);
            }
            else { //Received data
                if (rxBuffer.isFull()) {
                    sercom->prepareNackBitWIRE();
                }
                else {
                    //Store data
                    rxBuffer.store_char(sercom->readDataWIRE());

                    sercom->prepareAckBitWIRE();
                }

                sercom->prepareCommandBitsWire(0x03);
            }
        }
    }
}

// SERCOM4 is on pins A4 and A5. SDA: A4, SCL: A5.
I2C Wire(&sercom4, A4, A5);

void SERCOM4_0_Handler(void) { Wire.onService(); }
void SERCOM4_1_Handler(void) { Wire.onService(); }
void SERCOM4_2_Handler(void) { Wire.onService(); }
void SERCOM4_3_Handler(void) { Wire.onService(); }
