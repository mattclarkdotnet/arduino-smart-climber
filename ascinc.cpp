#include <Arduino_LSM9DS1.h>
#include <Arduino.h>
#include <math.h>

#include "ascinc.h"

boolean serial_debug_inc = false;
float scale_difficulty_by = 1.0;    // allows you to e.g. ride with Zwift at 50% difficulty while still getting full inclination adjustment
boolean fix_negative_grades = true; // Zwift halves negative grades regardless of difficulty setting, so we should double them

float current_target_inclination = 0.0;

/**
 * keep a moving average of HISTORY_LENGTH readings of the x reading of the accelerometer.  Keep the maths simple on each update.
 *
 */
#define HISTORY_LENGTH 100
float average_inclination_percent = 0.0;
float x, y, z;                                          // Last read acceleration values
float bike_inclination_history[HISTORY_LENGTH] = {0.0}; // Measured in percent
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
        average_inclination_percent = bike_inclination_history_average;
    }
    if (serial_debug_inc && Serial)
    {
        Serial.print("average_inclination_percent: ");
        Serial.println(average_inclination_percent);
    }
}

float getAverageInclinationPercent()
{
    return average_inclination_percent;
}

float getLatestInclinationPercent()
{
    // return the average of the last 10 readings in bike_inclination_history
    float latest_inclination_percent = 0.0;
    for (int i = 0; i < 10; i++)
    {
        latest_inclination_percent += bike_inclination_history[(bike_inclination_history_readIndex - i) % HISTORY_LENGTH];
    }
    latest_inclination_percent /= 10.0;
    return latest_inclination_percent;
}

void updateZwiftInclination(float newZInclination)
{
    if (fix_negative_grades && newZInclination < 0)
    {
        newZInclination = newZInclination * 2;
    }
    current_target_inclination = newZInclination * scale_difficulty_by;
}

float getTargetInclinationPercent()
{
    return current_target_inclination;
}
