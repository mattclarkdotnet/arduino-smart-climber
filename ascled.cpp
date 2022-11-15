#include <Arduino.h>

#include "ascled.h"

#define FULL_BRIGHTNESS 0
#define HALF_BRIGHTNESS 512
#define ZERO_BRIGHTNESS 1024

void setupLED()
{
    pinMode(LEDR, OUTPUT);
    pinMode(LEDB, OUTPUT);
    pinMode(LEDB, OUTPUT);
}

void setLEDColour(int red, int green, int blue)
{
    analogWrite(LEDR, red);
    analogWrite(LEDG, green);
    analogWrite(LEDB, blue);
}

void setLEDto(colours_t colour_name)
{
    switch (colour_name)
    {
    case RED:
        setLEDColour(FULL_BRIGHTNESS, ZERO_BRIGHTNESS, ZERO_BRIGHTNESS);
        break;
    case GREEN:
        setLEDColour(ZERO_BRIGHTNESS, FULL_BRIGHTNESS, ZERO_BRIGHTNESS);
        break;
    case BLUE:
        setLEDColour(ZERO_BRIGHTNESS, ZERO_BRIGHTNESS, FULL_BRIGHTNESS);
        break;
    case PURPLE:
        setLEDColour(FULL_BRIGHTNESS, ZERO_BRIGHTNESS, FULL_BRIGHTNESS);
        break;
    case ORANGE:
        setLEDColour(FULL_BRIGHTNESS, HALF_BRIGHTNESS, ZERO_BRIGHTNESS);
        break;
    case WHITE:
        setLEDColour(FULL_BRIGHTNESS, FULL_BRIGHTNESS, FULL_BRIGHTNESS);
        break;
    }
}