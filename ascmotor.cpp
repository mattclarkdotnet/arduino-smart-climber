#include <Arduino.h>

#include "ascmotor.h"
#include "ascled.h"
#include "mbed.h"

motor_state_t motor_state = ST_MOTOR_STOPPED;

// Use mbed OS functions to control PWM pin frequency and duty cycle instead of the default 500Hz
// Still havent actually found a good PWM approach yet but leaving this implementation in anyway
mbed::PwmOut pwmPin(digitalPinToPinName(D6));

void setupMotor()
{
    // Configure motor control pins and set both low, i.e. no motor movement
    pinMode(A6, OUTPUT);
    pinMode(A7, OUTPUT);
    digitalWrite(A6, LOW);
    digitalWrite(A7, LOW);

    // configure motor PWM control pin
    pwmPin.period(1.0 / 10); // PWM frequency - still struggling to find a good one
    pwmPin.write(1.0);       // output high disables the motor
    motor_state = ST_MOTOR_STOPPED;
}

void moveUp(colours_t colour)
{
    setLEDto(colour);
    pwmPin.write(0.0);
    digitalWrite(A6, LOW);
    digitalWrite(A7, HIGH);
    motor_state = ST_MOTOR_UP;
}

void moveDown(colours_t colour)
{
    setLEDto(colour);
    pwmPin.write(0.0);
    digitalWrite(A6, HIGH);
    digitalWrite(A7, LOW);
    motor_state = ST_MOTOR_DOWN;
}

void moveStop(colours_t colour)
{
    setLEDto(colour);
    pwmPin.write(1.0);
    digitalWrite(A6, LOW);
    digitalWrite(A7, LOW);
    motor_state = ST_MOTOR_STOPPED;
}

motor_state_t getMotorState()
{
    return motor_state;
}
