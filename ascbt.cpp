#include <Arduino.h>
#include <ArduinoBLE.h>

#include "ascbt.h"
#include "ascled.h"

boolean serial_debug_bt = true;
uint emulated_power = 0; // set to non-zero value to fake power data for debug purposes`

// This fmcp_data_t structure represents the BLE control point data. The first octet represents the opcode of the request
// followed by a parameter array of maximum 18 octects
typedef struct __attribute__((packed))
{
    uint8_t OPCODE;
    uint8_t OCTETS[FMCP_DATA_SIZE - 1];
} fmcp_data_t;

typedef union // The union type automatically maps the bytes member array to the fmcp_data_t structure member values
{
    fmcp_data_t values;
    uint8_t bytes[FMCP_DATA_SIZE];
} fmcp_data_ut;

// Indoor Bike Data characteristic variables
const uint16_t flagMoreData = 1;
const uint16_t flagAverageSpeed = 2;
const uint16_t flagInstantaneousCadence = 4;
const uint16_t flagAverageCadence = 8;
const uint16_t flagTotalDistance = 16;
const uint16_t flagResistanceLevel = 32;
const uint16_t flagInstantaneousPower = 64;
const uint16_t flagAveragePower = 128;
const uint16_t flagExpendedEnergy = 256;
const uint16_t flagHeartRate = 512;
const uint16_t flagMetabolicEquivalent = 1024;
const uint16_t flagElapsedTime = 2048;
const uint16_t flagRemainingTime = 4096;

// Fitness Machine Control Point opcodes
const uint8_t fmcpRequestControl = 0x00;
const uint8_t fmcpReset = 0x01;
const uint8_t fmcpSetTargetSpeed = 0x02;
const uint8_t fmcpSetTargetInclination = 0x03;
const uint8_t fmcpSetTargetResistanceLevel = 0x04;
const uint8_t fmcpSetTargetPower = 0x05;
const uint8_t fmcpSetTargetHeartRate = 0x06;
const uint8_t fmcpStartOrResume = 0x07;
const uint8_t fmcpStopOrPause = 0x08;
const uint8_t fmcpSetTargetedExpendedEngery = 0x09;
const uint8_t fmcpSetTargetedNumberOfSteps = 0x0A;
const uint8_t fmcpSetTargetedNumberOfStrided = 0x0B;
const uint8_t fmcpSetTargetedDistance = 0x0C;
const uint8_t fmcpSetTargetedTrainingTime = 0x0D;
const uint8_t fmcpSetTargetedTimeInTwoHeartRateZones = 0x0E;
const uint8_t fmcpSetTargetedTimeInThreeHeartRateZones = 0x0F;
const uint8_t fmcpSetTargetedTimeInFiveHeartRateZones = 0x10;
const uint8_t fmcpSetIndoorBikeSimulationParameters = 0x11;
const uint8_t fmcpSetWheelCircumference = 0x12;
const uint8_t fmcpSetSpinDownControl = 0x13;
const uint8_t fmcpSetTargetedCadence = 0x14;
const uint8_t fmcpResponseCode = 0x80;

// BLE variables
fmcp_data_ut fmcpData;
short fmcpValueLength;
volatile long lastControlPointEvent = 0;
long previousControlPointEvent = 0;
float target_inclination_percent = 0.0;

// Fitness Machine Service, uuid 0x1826 or 00001826-0000-1000-8000-00805F9B34FB
BLEService fitnessMachineService("1826"); // FTMS

// Service characteristics exposed by FTMS
BLECharacteristic fitnessMachineFeatureCharacteristic("2ACC", BLERead, 8);                                  // Fitness Machine Feature, mandatory, read
BLECharacteristic indoorBikeDataCharacteristic("2AD2", BLENotify, 8);                                       // Indoor Bike Data, optional, notify
BLECharacteristic supportedInclinationRangeCharacteristic("2AD5", BLERead, 6);                              // Supported Inclination, read, optional
BLECharacteristic fitnessMachineControlPointCharacteristic("2AD9", BLEWrite | BLEIndicate, FMCP_DATA_SIZE); // Fitness Machine Control Point, optional, write & indicate
BLECharacteristic fitnessMachineStatusCharacteristic("2ADA", BLENotify, 2);                                 // Fitness Machine Status, mandatory, notify

// Buffers used to write to the characteristics and initial values
unsigned char ftmfBuffer[4] = {0b10001001, 0b00000000, 0b00000010, 0b00000000}; // FM Features: 3 (Inclination), plus 0 (Avrage speed, to placate zwift).  FM Target setting features: 1 (Inclination)
unsigned char ibdBuffer[8] = {0, 0, 0, 0, 0, 0, 0, 0};                          // Needs to be large enough to accomodate all data we might send - in practice we only send power in debug mode
unsigned char sirBuffer[6] = {0x9C, 0xFF, 0xC8, 0x00, 0x0A, 0x00};              // Supported Inclination Range - Min, Max, Min increment (sint16, sint16, uint16) units are 0.1%, so 100=10%
unsigned char ftmsBuffer[2] = {0, 0};
unsigned char tsBuffer[2] = {0x0, 0x0}; // Training status: flags: 0 (no string present); Status: 0x00 = Other
unsigned char ftmcpBuffer[20];

void setupBLE()
{
    BLE.setDeviceName(DEVICE_NAME_LONG);
    BLE.setLocalName(DEVICE_NAME_SHORT);
    BLE.setAdvertisedService(fitnessMachineService);
    fitnessMachineService.addCharacteristic(fitnessMachineFeatureCharacteristic);
    fitnessMachineService.addCharacteristic(indoorBikeDataCharacteristic);
    fitnessMachineService.addCharacteristic(supportedInclinationRangeCharacteristic);
    fitnessMachineService.addCharacteristic(fitnessMachineControlPointCharacteristic);
    fitnessMachineService.addCharacteristic(fitnessMachineStatusCharacteristic);
    BLE.addService(fitnessMachineService);

    // Write values to the characteristics that can be read
    fitnessMachineFeatureCharacteristic.writeValue(ftmfBuffer, 4);
    indoorBikeDataCharacteristic.writeValue(ibdBuffer, 8);
    supportedInclinationRangeCharacteristic.writeValue(sirBuffer, 6);
    fitnessMachineStatusCharacteristic.writeValue(ftmsBuffer, 2);
    // trainingStatusCharacteristic.writeValue(tsBuffer, 2);

    // Write requests to the control point characteristic are handled by an event handler
    fitnessMachineControlPointCharacteristic.setEventHandler(BLEWritten, fitnessMachineControlPointCharacteristicWritten);

    // Add event handlers for connection and discoinnection events, so we can update LED colours etc
    BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
    BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);
}

float getTargetInclinationPercent()
{
    return target_inclination_percent;
}

/**
 * Handles an incoming Fitness Machine Control Point request
 *
 * @return void
 */
void handleControlPoint()
{
    if (serial_debug_bt && Serial)
    {
        Serial.print("Control point received, opcode: 0x");
        Serial.println(fmcpData.values.OPCODE, HEX);
    }
    switch (fmcpData.values.OPCODE)
    {
    case fmcpRequestControl:
    {
        if (serial_debug_bt && Serial)
        {
            Serial.println("Allowing control");
        }
        writeFTMCPSuccess();
        break;
    }
    case fmcpStartOrResume:
    {
        if (serial_debug_bt && Serial)
        {
            Serial.println("Setting training status to RUNNING");
        }
        writeFTMCPSuccess();
        break;
    }
    case fmcpStopOrPause:
    {
        if (serial_debug_bt && Serial)
        {
            Serial.println("Setting training status to STOPPED");
        }
        writeFTMCPSuccess();
        break;
    }
    case fmcpSetIndoorBikeSimulationParameters:
    {
        if (serial_debug_bt && Serial)
        {
            Serial.println("Setting indoor bike simulation parameters");
        }
        short gr = (fmcpData.values.OCTETS[3] << 8) + fmcpData.values.OCTETS[2]; // Short is 16 bit signed, so a negative grade is correctly converted from two bytes to signed value. Highest bit is sign bit
        target_inclination_percent = gr / 100;
        if (serial_debug_bt && Serial)
        { // As sent by Zwift, so may be scaled according to the "diffculty" setting, and will always be halved for descents
            Serial.print("Inclination: raw ");
            Serial.print(gr);
            Serial.print(" = ");
            Serial.print(target_inclination_percent);
            Serial.println("%");
        }
        writeFTMCPSuccess();
        break;
    }
    case fmcpSetTargetInclination:
    {
        short in = (fmcpData.values.OCTETS[1] << 8) + fmcpData.values.OCTETS[0]; // Short is 16 bit signed, so a negative grade is correctly converted from two bytes to signed value. Highest bit is sign bit
        target_inclination_percent = in / 100.0;
        if (serial_debug_bt && Serial)
        { // As sent by Zwift, so may be scaled according to the "diffculty" setting, and will always be halved for descents
            Serial.print("Setting target inclination: ");
            Serial.println(target_inclination_percent);
        }
        writeFTMCPSuccess();
        break;
    }
    case fmcpReset:
    case fmcpSetTargetResistanceLevel:
    case fmcpSetTargetSpeed:
    case fmcpSetTargetPower:
    case fmcpSetTargetHeartRate:
    case fmcpSetTargetedExpendedEngery:
    case fmcpSetTargetedNumberOfSteps:
    case fmcpSetTargetedNumberOfStrided:
    case fmcpSetTargetedDistance:
    case fmcpSetTargetedTrainingTime:
    case fmcpSetTargetedTimeInTwoHeartRateZones:
    case fmcpSetTargetedTimeInThreeHeartRateZones:
    case fmcpSetTargetedTimeInFiveHeartRateZones:
    case fmcpSetWheelCircumference:
    case fmcpSetSpinDownControl:
    case fmcpSetTargetedCadence:
    {
        writeFTMCPFailure();
        break;
    }
    }
}

/**
 * Writes the Indoor Bike Data characteristic
 *
 * @return void
 */
void writeIndoorBikeDataCharacteristic()
{

    ibdBuffer[0] = 0x01 | flagInstantaneousPower; // bit 0 = 1 (instantaneous speed not present), bit 6 = 1: instantaneous cadence present
    ibdBuffer[1] = 0x00;                          // unused
    ibdBuffer[2] = 0x00;                          // first characteristic low byte
    ibdBuffer[3] = 0x00;                          // first characteristic high byte
    ibdBuffer[4] = 0x00;
    ibdBuffer[5] = 0x00;
    ibdBuffer[6] = 0x00;
    ibdBuffer[7] = 0x00;

    if (emulated_power > 0)
    {
        ibdBuffer[2] = (int)round(emulated_power) & 0xFF; // Instantaneous Power, uint16
        ibdBuffer[3] = ((int)round(emulated_power) >> 8) & 0xFF;
        indoorBikeDataCharacteristic.writeValue(ibdBuffer, 8);
    }
}

/**
 * write FTMCP success code into response, using the most recently received OpCode
 *
 * @return void
 */
void writeFTMCPSuccess()
{
    ftmcpBuffer[0] = fmcpResponseCode;
    ftmcpBuffer[1] = fmcpData.values.OPCODE;
    ftmcpBuffer[2] = 0x01;
    fitnessMachineControlPointCharacteristic.writeValue(ftmcpBuffer, 3);
}

/**
 * write FTMCP failure code into response, using the most recently received OpCode
 *
 * @return void
 */
void writeFTMCPFailure()
{
    ftmcpBuffer[0] = fmcpResponseCode;
    ftmcpBuffer[1] = fmcpData.values.OPCODE;
    ftmcpBuffer[2] = 0x02;
    fitnessMachineControlPointCharacteristic.writeValue(ftmcpBuffer, 3);
    if (serial_debug_bt && Serial)
    {
        Serial.print("Unsupported OpCode received: ");
        Serial.println(fmcpData.values.OPCODE);
    }
}

/**
 * Fitness Machine Control Point that is written to by the BLE client (e.g. Zwift)
 *
 *  @return void
 */
void fitnessMachineControlPointCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic)
{
    fmcpValueLength = fitnessMachineControlPointCharacteristic.valueLength();
    memset(fmcpData.bytes, 0, sizeof(fmcpData.bytes));
    fitnessMachineControlPointCharacteristic.readValue(fmcpData.bytes, fmcpValueLength);
    lastControlPointEvent = millis();
}

boolean freshEvent()
{
    if (previousControlPointEvent != lastControlPointEvent)
    {
        previousControlPointEvent = lastControlPointEvent;
        return true;
    }
}

/**
 * Lights the internal RGB LED to blue on connection
 *
 * @return void
 */
void blePeripheralConnectHandler(BLEDevice central)
{
    if (serial_debug_bt && Serial)
    {
        Serial.println("Client connected");
    }
}

/*
 * Lights the internal RGB LED to green on disconnection
 *
 * @return void
 */
void blePeripheralDisconnectHandler(BLEDevice central)
{
    if (serial_debug_bt && Serial)
    {
        Serial.println("Client disconnected");
    }
}