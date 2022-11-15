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

/**
 * keep a moving average of ZINC_HISTORY_LENGTH readings of the inclination sent by Zwift
 * these are quite fine grained, and read out every 10ms or so (per the main Arduino loop)
 * so long as Zwift updates us that often.  All a bit empirical when it comes to choosing
 * a history length
 *
 */
#define ZINC_HISTORY_LENGTH 25
float zinc_history[ZINC_HISTORY_LENGTH];
int zinc_historyIndex = 0;
float zinc_historySum = 0.0;
float zinc_historyAverage = 0.0;

void updateZwiftInclinationHistory(float newZInclination)
{
    zinc_historySum = zinc_historySum - x_history[zinc_historyIndex] + newZInclination;
    zinc_history[zinc_historyIndex] = x;
    zinc_historyIndex = (zinc_historyIndex + 1) % ZINC_HISTORY_LENGTH;
    zinc_historyAverage = zinc_historySum / ZINC_HISTORY_LENGTH;
}

float getTargetInclinationPercent()
{
    return zinc_historyAverage;
}
