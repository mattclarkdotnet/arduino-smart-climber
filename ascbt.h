#include <ArduinoBLE.h>

#define FMCP_DATA_SIZE 19 // Control point consists of 1 opcode (byte) and maximum 18 bytes as parameters
#define DEVICE_NAME_LONG "Arduino Smart Climber"
#define DEVICE_NAME_SHORT "ASCL2"

void fitnessMachineControlPointCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic);
void blePeripheralConnectHandler(BLEDevice central);
void blePeripheralDisconnectHandler(BLEDevice central);
void setupBLE();
void writeFTMCPSuccess();
void writeFTMCPFailure();
void writeFTMCPResponse(uint8_t responseCode, uint8_t responseValue);
void writeIndoorBikeDataCharacteristic();
boolean freshEvent();
float getTargetInclinationPercent();
void handleControlPoint();
