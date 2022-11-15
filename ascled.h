enum colours_t // specific LED colours that are used to indicate state
{
    RED,    // Only used in flashing red/green to indicate an error
    GREEN,  // BT connected to client
    BLUE,   // BT active but not connected
    PURPLE, // Performing setup
    ORANGE,
    WHITE, // Levelling mode
};

void setupLED();
void setLEDColour(int red, int green, int blue);
void setLEDto(colours_t colour_name);