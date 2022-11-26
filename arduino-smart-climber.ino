#include <Arduino_LSM9DS1.h>

#include "ascled.h"
#include "ascbt.h"
#include "ascinc.h"
#include "ascmotor.h"
#include "asctrainer.h"

boolean serial_debug = true;
long last_debug_millis = 0;
#define SERIAL_SPEED 115200
#define NOTIFICATION_INTERVAL 1000 // send bike data notifications to Zwift every 1 second
uint16_t emulated_power = 100;     // set to non-zero value to fake power data for debug purposes
uint16_t emulated_cadence = 70;    // set to non-zero value to fake cadence data for debug purposes

enum sm_state_t // defined states (see statemachine.dot for the transition diagram)
{
  SM_STATE_STARTING,
  SM_STATE_LEVELLING,
  SM_STATE_RUNNING,
  SM_STATE_ERROR
};

// State management variables
long current_millis;
long last_millis;
int sm_state;
long previous_notification_millis = 0;
long last_motor_stop_millis = 0;

// The reading of the inclinometer after levelling
float reference_inclination_percent = 0.0;

/**
 * Arduino setup function, runs once
 *
 * @return void
 */
void setup()
{
  // Configure onboard LED and set to purple
  setupLED();
  setLEDto(PURPLE);
  sm_state = SM_STATE_STARTING;

  // Configure pushbutton input pins
  pinMode(D2, INPUT);
  pinMode(D3, INPUT);
  pinMode(D4, INPUT);

  // Configure the motor pins and state,
  setupMotor();

  // Serial setup
  if (serial_debug)
  {
    Serial.begin(SERIAL_SPEED);
    int serial_wait_timer = millis();
    while (!Serial && (millis() - serial_wait_timer < 10000))
    {
      delay(100); // wait for serial port to connect. Needed for native USB port only
    }
    if (Serial)
    {
      Serial.print(DEVICE_NAME_LONG);
      Serial.print(" (");
      Serial.print(DEVICE_NAME_SHORT);
      Serial.println(")");
    }
  }

  // Accelerometer setup
  if (!IMU.begin())
  {
    if (serial_debug && Serial)
    {
      Serial.println("Error initiliazing IMU, please restart");
    }
    sm_state = SM_STATE_ERROR;
    return;
  }

  if (serial_debug && Serial)
  {
    Serial.print("Accelerometer sample rate = ");
    Serial.print(IMU.accelerationSampleRate());
    Serial.println("Hz");
  }

  // Bluetooth setup
  if (!BLE.begin())
  {
    if (serial_debug && Serial)
    {
      Serial.println("Error initiliazing bluetooth module, please restart");
    }
    sm_state = SM_STATE_ERROR;
    return;
  }
  setupBLE();
  sm_state = SM_STATE_LEVELLING;
}

/**
 * Arduino loop, runs continuously
 *
 * @return void
 */
void loop()
{
  // Timing and loop housekeeping
  delay(10);
  last_millis = current_millis;
  current_millis = millis();
  boolean loop_debug = false;
  if (current_millis - last_debug_millis > 1000)
  {
    last_debug_millis = current_millis;
    loop_debug = true;
  }

  // Track physical bike inclination regardless of state
  updateBikeInclination(current_millis - last_millis);

  // Manage bluetooth regardless of state
  boolean cp_handled;
  boolean central_connected = centralConnected();
  boolean peripheral_connected = peripheralConnected();
  boolean bt_connected = central_connected && peripheral_connected;

  // Read an update from Zwift and store resistance level and inclination in the cp data structure.
  // This function checks internally to avoid processing the same control point data twice
  // It needs to run frequently as Zwift updates much more often than once per second
  if (central_connected)
  {
    cp_handled = handleControlPoint();
  }

  // If we have been updated by Zwift, or in any event at least once a second:
  // update the trainer with target resistance, get new trainer data and update zwift with it
  if (cp_handled || (current_millis > previous_notification_millis + NOTIFICATION_INTERVAL))
  {
    previous_notification_millis = current_millis;
    if (peripheral_connected)
    {
      // Update the trainer target resistance
      writeTrainerTargetResistance();
      // Fetch info from the trainer
      readTrainerPowerAndCadence();
    }
    // send a notification to Zwift every NOTIFICATION_INTERVAL milliseconds, otherwise Zwift will show "no signal"
    writeIndoorBikeDataCharacteristic(emulated_cadence, emulated_power); // replace emulated with real once that code is working
  }

  // Other behaviours dependent on the state we are in
  switch (sm_state)
  {
  case SM_STATE_ERROR:
  {
    // Blink the LED to show error condition
    while (1)
    {
      setLEDto(AQUA);
      delay(250);
      setLEDto(RED);
      delay(250);
    }
    break;
  }
  case SM_STATE_LEVELLING:
  {
    // Move the bike up and down while one of the up/down buttons is pressed, otherwise stop
    // TODO: the control loop isn't great, perhaps slow down the input from Zwift since control is binary
    if (digitalRead(D3) == HIGH)
    {
      moveDown(GREEN);
    }
    else if (digitalRead(D4) == HIGH)
    {
      moveUp(RED);
    }
    else
    {
      moveStop(bt_connected ? WHITE : YELLOW);
      if (digitalRead(D2) == HIGH)
      {
        // User has pressed the levelling button. Stop all motion, record the reference inclination and move on to the next state
        while (digitalRead(D2) == HIGH)
        {
          delay(100); // to debounce the D2 keypress, otherwise we'll end up right back in SM_STATE_LEVELLING
        }
        reference_inclination_percent = getAverageInclinationPercent(); // set the reference inclination to the current inclination
        sm_state = SM_STATE_RUNNING;
        if (serial_debug && Serial)
        {
          Serial.print("User has levelled the bike, reference inclination is ");
          Serial.print(reference_inclination_percent);
          Serial.println("%");
          Serial.println("Entering SM_STATE_RUNNING");
        }
      }
    }
    break;
  }
  case SM_STATE_RUNNING:
  {
    if (digitalRead(D2) == HIGH)
    {
      // User has pressed the levelling button durring normal operation, return to levelling mode
      moveStop(bt_connected ? BLUE : AQUA);
      while (digitalRead(D2) == HIGH)
      {
        delay(100); // to debounce the D2 keypress, otherwise we'll end up right back in SM_STATE_RUNNING
      }
      sm_state = SM_STATE_LEVELLING;
      if (serial_debug && Serial)
      {
        Serial.println("Entering SM_STATE_LEVELLING");
      }
      break;
    }
    else
    {
      float target_inclination_percent = getTargetInclinationPercent();
      float average_inc_diff = target_inclination_percent - (getAverageInclinationPercent() - reference_inclination_percent);
      float latest_inc_diff = target_inclination_percent - (getLatestInclinationPercent() - reference_inclination_percent);
      motor_state_t motor_state = getMotorState();
      // By default, aim for a maximum difference to target of 1% and use the average (slower but more accurate) difference.
      // If we haven't moved in a while, narrow the target difference to 0.5%
      // If the motor is already moving, use the default target difference but use the latest (faster but less accurate) difference.
      // Consider this all to be a very crude PID controller with the benefit of being very simple.
      float run_diff = 1.0;
      float use_diff = average_inc_diff;
      if ((motor_state == ST_MOTOR_STOPPED) && last_motor_stop_millis && (current_millis - last_motor_stop_millis > 2000))
      {
        run_diff = 0.5; // haven't moved for a bit, try to get a bit closer to the target
      }
      else if (motor_state == ST_MOTOR_UP || motor_state == ST_MOTOR_DOWN)
      {
        use_diff = latest_inc_diff; // we're moving, use the quicker-changing but noiser inclination measure
      }
      if (loop_debug && serial_debug && Serial)
      {
        Serial.print("run_diff=");
        Serial.print(run_diff);
        Serial.print(", use_diff=");
        Serial.println(use_diff);
      }
      // Switch the motor direction according to the inputs set above
      if (use_diff >= run_diff)
      {
        moveUp(RED);
      }
      else if (use_diff <= -run_diff)
      {
        moveDown(GREEN);
      }
      else
      {
        if (motor_state == ST_MOTOR_UP || motor_state == ST_MOTOR_DOWN)
        {
          last_motor_stop_millis = current_millis;
        }
        moveStop(bt_connected ? BLUE : AQUA);
      }
    }
  }
  }
}