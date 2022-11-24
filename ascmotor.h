#pragma once

#include "ascled.h"

enum motor_state_t
{
    ST_MOTOR_STOPPED,
    ST_MOTOR_UP,
    ST_MOTOR_DOWN
};

void setupMotor();
void moveUp(colours_t colour);
void moveDown(colours_t colour);
void moveStop(colours_t colour);
motor_state_t getMotorState();