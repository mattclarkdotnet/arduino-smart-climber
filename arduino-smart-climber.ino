#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>

#include "ascled.h"
#include "ascbt.h"
#include "ascinc.h"

boolean serial_debug = true;
#define SERIAL_SPEED 115200
#define NOTIFICATION_INTERVAL 1000

enum sm_state_t // defined states (see statemachine.dot for the transition diagram)
{
  SM_STATE_STARTING,
  SM_STATE_LEVELLING,
  SM_STATE_CONNECTING,
  SM_STATE_RUNNING,
  SM_STATE_ERROR
};

// State management variables
long current_millis;
long last_millis;
int sm_state;
long previous_notification_millis = 0;
BLEDevice central;

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

  // Configure motor control pins and set both low, i.e. no motor movement
  pinMode(A6, OUTPUT);
  pinMode(A7, OUTPUT);
  digitalWrite(A6, LOW);
  digitalWrite(A7, LOW);

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

void moveUp(colours_t colour)
{
  setLEDto(colour);
  digitalWrite(A6, LOW);
  digitalWrite(A7, HIGH);
}

void moveDown(colours_t colour)
{
  setLEDto(colour);
  digitalWrite(A6, HIGH);
  digitalWrite(A7, LOW);
}

void moveStop(colours_t colour)
{
  setLEDto(colour);
  digitalWrite(A6, LOW);
  digitalWrite(A7, LOW);
}

/**
 * Arduino loop, runs continuously
 *
 * @return void
 */
void loop()
{
  delay(10); // Really no point in doing everything more than 100 times per second
  last_millis = current_millis;
  current_millis = millis();
  updateBikeInclination(current_millis - last_millis); // keep track of bike inclination all the time
  // Always be alive so we stay connected
  BLE.poll();
  central = BLE.central();
  if (central && central.connected())
  {
    if (current_millis > previous_notification_millis + NOTIFICATION_INTERVAL)
    {
      // send a notification every NOTIFICATION_INTERVAL milliseconds, otherwise Zwift will show "no signal"
      // There's no indoor bike data field for inclination, so fudging by sending a 0 power field (or the emulated power if that's set)
      previous_notification_millis = current_millis;
      writeIndoorBikeDataCharacteristic();
    }
    handleControlPoint(); // checks internally to avoid processing the same control point data twice
  }

  switch (sm_state)
  {
  case SM_STATE_ERROR:
  {
    // Blink the LED to show error condition
    while (1)
    {
      setLEDto(ORANGE);
      delay(250);
      setLEDto(RED);
      delay(250);
    }
    break;
  }
  case SM_STATE_LEVELLING:
  {
    // Update inclination while waiting for the user to level the bike
    setLEDto(WHITE);
    if (digitalRead(D2) == HIGH)
    {
      // User has pressed the levelling button. Stop all motion, record the reference inclination and move on to the next state
      moveStop(WHITE);
      reference_inclination_percent = getCurrentInclinationPercent(); // set the reference inclination to the current inclination
      sm_state = SM_STATE_RUNNING;
      while (digitalRead(D2) == HIGH)
      {
        delay(100); // to debounce the D2 keypress, otherwise we'll end up right back in SM_STATE_LEVELLING
      }
      if (serial_debug && Serial)
      {
        Serial.print("User has levelled the bike, reference inclination is ");
        Serial.print(reference_inclination_percent);
        Serial.println("%");
        Serial.println("Entering SM_STATE_RUNNING");
      }
    }
    else
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
        moveStop(WHITE);
      }
    }
    break;
  }
  case SM_STATE_RUNNING:
  {
    setLEDto(BLUE);
    if (digitalRead(D2) == HIGH)
    {
      // User has pressed the levelling button durring normal operation, return to levelling mode
      moveStop(BLUE);
      sm_state = SM_STATE_LEVELLING;
      while (digitalRead(D2) == HIGH)
      {
        delay(100); // to debounce the D2 keypress, otherwise we'll end up right back in SM_STATE_RUNNING
      }
      if (serial_debug && Serial)
      {
        Serial.println("Entering SM_STATE_LEVELLING");
      }
      break;
    }
    else
    {
      float target_inclination_percent = getTargetInclinationPercent();
      float current_inclination_percent = getCurrentInclinationPercent();
      float inc_diff = target_inclination_percent - (current_inclination_percent - reference_inclination_percent);
      if (inc_diff >= 1)
      {
        moveUp(RED);
      }
      else if (inc_diff <= -1)
      {
        moveDown(GREEN);
      }
      else
      {
        moveStop(BLUE);
      }
    }
  }
  }
}
