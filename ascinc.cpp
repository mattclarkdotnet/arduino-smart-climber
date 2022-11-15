#include <Arduino_LSM9DS1.h>
#include <Arduino.h>
#include <math.h>

#include "ascinc.h"

#define HISTORY_LENGTH 25 // how many accelerometer data points we'll keep in history
boolean serial_debug_inc = false;

// globals for keeping track of inclination of the device
float x, y, z;
float candidate_inclination = 0;
float current_inclination = 0;

float x_history[HISTORY_LENGTH] = {0.0}; // Measured in g
int x_history_readIndex = 0;             // the index of the current reading
float x_history_total = 0.0;             // the running total
float x_history_average = 0.0;           // the average

/**
 * keep a moving average of HISTORY_LENGTH readings of the x reading of the accelerometer.  Keep the maths simple on each update.
 *
 */
void updateInclinationHistory()
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
    if (abs(candidate_inclination - current_inclination) > 1)
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
