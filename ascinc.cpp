#include <Arduino_LSM9DS1.h>
#include <Arduino.h>
#include <math.h>

#include "ascinc.h"

boolean serial_debug_inc = false;

/**
 * keep a moving average of HISTORY_LENGTH readings of the x reading of the accelerometer.  Keep the maths simple on each update.
 *
 */
#define HISTORY_LENGTH 100
float current_inclination_percent = 0.0;
float x, y, z;                                          // Last read acceleration values
float bike_inclination_history[HISTORY_LENGTH] = {0.0}; // Measured in radians
int bike_inclination_history_readIndex = 0;             // the index of the current reading
float bike_inclination_history_total = 0.0;             // the running total
float bike_inclination_history_average = 0.0;           // the average

void updateBikeInclination(long delta_t)
{
    if (IMU.accelerationAvailable())
    {
        IMU.readAcceleration(x, y, z); // x will be downward acceleration in the x axis in g.  0 = horizontal, 1 = end up, -1 = end down
        float inclination_percent = tan(asin(x)) * 100.0;
        bike_inclination_history_total = bike_inclination_history_total - bike_inclination_history[bike_inclination_history_readIndex] + inclination_percent;
        bike_inclination_history[bike_inclination_history_readIndex] = inclination_percent;
        bike_inclination_history_readIndex = (bike_inclination_history_readIndex + 1) % HISTORY_LENGTH;
        bike_inclination_history_average = bike_inclination_history_total / HISTORY_LENGTH;
        current_inclination_percent = bike_inclination_history_average;
    }
    if (serial_debug_inc && Serial)
    {
        Serial.print("current_inclination_percent: ");
        Serial.println(current_inclination_percent);
    }
}

float getCurrentInclinationPercent()
{
    return current_inclination_percent;
}

float last_zwift_inclination_update = 0.0;
float current_target_inclination = 0.0;
long last_zwift_inclination_update_millis = 0;
long last_target_inclination_update_millis = 0;

void updateZwiftInclination(float newZInclination)
{
    long now_millis = millis();
    last_zwift_inclination_update = newZInclination;
    last_zwift_inclination_update_millis = now_millis;
    if ((abs(last_zwift_inclination_update - current_target_inclination) >= 1) || (now_millis - last_target_inclination_update_millis < 3000))
    {
        // update the target if the deviation is at least 1%, or if we haven't made an update for 3 seconds
        current_target_inclination = newZInclination;
        last_target_inclination_update_millis = now_millis;
        if (serial_debug_inc && Serial)
        {
            Serial.print("Target inclination changed: ");
            Serial.print(current_target_inclination);
            Serial.println("%");
        }
    }
}

float getTargetInclinationPercent()
{
    return current_target_inclination;
}
