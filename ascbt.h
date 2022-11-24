#pragma once

#include <ArduinoBLE.h>

#define FMCP_DATA_SIZE 19 // Control point consists of 1 opcode (byte) and maximum 18 bytes as parameters
#define DEVICE_NAME_LONG "Arduino Smart Climber"
#define DEVICE_NAME_SHORT "ASCL2"

// Public interface
void setupBLE();
void writeIndoorBikeDataCharacteristic(uint fakePower);
void handleControlPoint();
boolean btConnected();

// Private functions
void fitnessMachineControlPointCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic);
void blePeripheralConnectHandler(BLEDevice central);
void blePeripheralDisconnectHandler(BLEDevice central);
void writeFTMCPSuccess();
void writeFTMCPFailure();
void writeFTMCPResponse(uint8_t responseCode, uint8_t responseValue);
