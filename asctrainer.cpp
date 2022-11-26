#include <Arduino.h>
#include <ArduinoBLE.h>

#include "asctrainer.h"

boolean serial_debug_trainer = true;

BLEDevice peripheral; // i.e. the trainer we'll proxy for

boolean peripheralConnected()
{
    if (!(peripheral && peripheral.connected()))
    {
        BLE.poll();
        if (serial_debug_trainer && Serial)
        {
            Serial.print("Scanning for trainer ");
            Serial.println(TRAINER_NAME);
        }
        BLE.scanForName(TRAINER_NAME);
        peripheral = BLE.available();
        BLE.stopScan();
        if (!peripheral)
        {
            if (serial_debug_trainer && Serial)
            {
                Serial.println("Trainer not found");
            }
            return false;
        }
        if (serial_debug_trainer && Serial)
        {
            Serial.println("Trainer found");
        }
        boolean trainer_connected = peripheral.connect();
        if (!trainer_connected)
        {
            if (serial_debug_trainer && Serial)
            {
                Serial.println("Trainer did not connect");
            }
            return false;
        }
        if (serial_debug_trainer && Serial)
        {
            Serial.println("Connected to trainer");
        }
    }
    // We would have bailed out early with "false" unless everything worked
    return true;
}

void writeTrainerTargetResistance()
{
}

void readTrainerPowerAndCadence()
{
}