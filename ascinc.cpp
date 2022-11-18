#include <Arduino_LSM9DS1.h>
#include <Arduino.h>
#include <math.h>

#include "ascinc.h"

boolean serial_debug_inc = false;

/**
 * keep a moving average of HISTORY_LENGTH readings of the x reading of the accelerometer.  Keep the maths simple on each update.
 *
 */
float x, y, z;
float candidate_inclination = 0;
float current_inclination = 0;

#define HISTORY_LENGTH 25                // how many accelerometer data points we'll keep in history
float x_history[HISTORY_LENGTH] = {0.0}; // Measured in g
int x_history_readIndex = 0;             // the index of the current reading
float x_history_total = 0.0;             // the running total
float x_history_average = 0.0;           // the average

void updateBikeInclinationHistory()
{
    if (IMU.accelerationAvailable())
    {
        IMU.readAcceleration(x, y, z); // x will be downward acceleration in the x axis in g.  0 = horizontal, 1 = end up, -1 = end down
        x_history_total = x_history_total - x_history[x_history_readIndex] + x;
        x_history[x_history_readIndex] = x;
        x_history_readIndex = (x_history_readIndex + 1) % HISTORY_LENGTH;
        x_history_average = x_history_total / HISTORY_LENGTH;
    }
}

/**
 * Calculate the current inclination of the device in degrees
 *
 */
float getCurrentInclinationPercent()
{
    // convert g to slope
    candidate_inclination = tan(asin(x_history_average)) * 100; // convert g to slope in %
    if (abs(candidate_inclination - current_inclination) >= 1)
    {
        current_inclination = candidate_inclination;
        if (serial_debug_inc && Serial)
        {
            Serial.print("Inclination changed by at least 1%: ");
            Serial.print(current_inclination);
            Serial.println("%");
        }
    }
    return current_inclination;
}

float last_zwift_inclination_update = 0.0;
float current_target_inclination = 0.0;
long last_zwift_inclination_update_millis = 0;
long last_target_inclination_update_millis = 0;

void updateZwiftInclinationHistory(float newZInclination)
{
    last_zwift_inclination_update = newZInclination;
    last_zwift_inclination_update_millis = millis();
    if (abs(last_zwift_inclination_update - current_target_inclination) >= 1)
    {
        // always update the target if the deviation is at least 1%
        current_target_inclination = newZInclination;
        last_target_inclination_update_millis = millis();
        if (serial_debug_inc && Serial)
        {
            Serial.print("Zwift inclination changed by at least 1%: ");
            Serial.print(current_target_inclination);
            Serial.println("%");
        }
    }
}

float getTargetInclinationPercent()
{
    if (millis() - last_target_inclination_update_millis < 3000)
    {
        // We made an update to the target inclination recently, stick with that
        return current_target_inclination;
    }
    else
    {
        // The target hasn't been updated for 3 seconds, either because it hasn't changed or because the change was less than 1%
        // so update the current target to the last one sent by Zwift anyway.  This will smooth out minor bumbs and stop the bike
        // moving up and down for tiny deviations, while also making it accurate if the deviations are long lived.
        current_target_inclination = last_zwift_inclination_update;
        return current_target_inclination;
    }
}
