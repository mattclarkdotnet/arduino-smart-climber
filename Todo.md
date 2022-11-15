# Todo list

* Fix the RGB status mess
* Write control code for motor driver
* Make the Nano use the inclination on zero confirmation as its baseline
* Draw real schematics
* Write procedure for levelling
  * Power up the climber
  * Use toggle switch to move front axle up/down until level (use spirit level or tape measure)
  * Press zero-confirm switch
* Figure out connections between power, motor, motor board and nano board
  * power -> motor board                Vin, GND
  * motor moard -> motor                Vout1, Vout2
  * motor board -> nano board           +12v, GND
  * nano board -> motor board           3v3, IN1, IN2
* Hook the Nano and the Driver up, with hardwired buttons for up/down levelling and a separate button for zero confirmation
* Attach the nano up to the bike

# Hardware to buy

* S1350 DPDT toggle switch
* S1098 SPST momentary pushbutton switch green
* Heatsink 8x8mm
* Z0510 7812 12V regulator
* W2360 6 core cable
* W2109 Figure 8 power cable (5A)
* P0633 2.5mm DC power plug
* P0621A 2.5mm DC power socket PCB mount
* Jumpers & pins