#pragma once

enum colours_t // specific LED colours that are used to indicate state
{
    PURPLE, // Performing Arduino setup
    WHITE,  // BT connected in SM_STATE_LEVELLING
    YELLOW, // BT not connected in SM_STATE_LEVELLING
    BLUE,   // BT connected in SM_STATE_RUNNING
    AQUA,   // BT not connected in SM_STATE_RUNNING
    RED,    // Moving up
    GREEN,  // Moving down
};

void setupLED();
void setLEDColour(int red, int green, int blue);
void setLEDto(colours_t colour_name);