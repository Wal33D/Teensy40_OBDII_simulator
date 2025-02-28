/*
  Teensy 4.0 OBDII CAN-BUS ECU simulator.
  www.skpang.co.uk
  December 2022

  Enhanced version:
   - Randomly selects a VIN from an array on SW2 press.
   - Randomizes INTAKE_AIR_TEMP and ENGINE_LOAD values.
   - Flashes LEDs for visual feedback.
   - ECU simulation logic moved into separate files (ecu_sim.* and can_helpers.*)
*/

#include <Arduino.h>
#include "ecu_sim.h"   // Contains the ECU simulation class with the new functionality

IntervalTimer timer;
  
// Global timing variables for the main loop
uint16_t led_tick = 0;
uint16_t pot_tick = 0;
uint16_t flash_led_tick = 0;

int led = 13;   // On-board LED on Teensy

// Called every 1ms via an interrupt timer
void tick(void)
{
    led_tick++;
    pot_tick++;
    flash_led_tick++;
}

void setup() {
  // Initialize LEDs (LED_red and LED_green are defined in ecu_sim.h)
  pinMode(led, OUTPUT);
  pinMode(LED_red, OUTPUT);
  pinMode(LED_green, OUTPUT);

  // Simple blink sequence at startup
  digitalWrite(LED_red, HIGH);
  delay(1000);
  digitalWrite(LED_red, LOW);
  digitalWrite(LED_green, HIGH);
  delay(1000);
  digitalWrite(LED_green, LOW);
  digitalWrite(LED_red, HIGH);
  delay(1000);
  digitalWrite(LED_red, LOW);
  digitalWrite(LED_green, HIGH);
  delay(1000);
  digitalWrite(LED_green, LOW);

  Serial.begin(115200);
  Serial.println("****** Teensy 4.0 OBDII Simulator - Enhanced ******");

  // Initialize the ECU simulation code (this now includes randomization functionality)
  ecu_sim.init(500000);

  // Start a 1ms interrupt timer
  timer.begin(tick, 1000);    // 1ms = 1000 microseconds
}

void loop() {
  // Process incoming CAN messages and respond via the ECU simulation
  ecu_sim.update();

  // Blink on-board LED every ~2 seconds
  if (led_tick > 2000) {
    led_tick = 0;
    digitalToggle(led);
  }

  // Update sensor (potentiometer) values approximately every 10ms.
  // This function also handles SW1 and SW2 button events (toggling DTC, randomizing VIN/sensor values)
  if (pot_tick > 10) {
    pot_tick = 0;
    ecu_sim.update_pots();
  }

  // Turn off green LED after ~50ms if it was lit by a CAN message
  if (flash_led_tick > 50) {
    flash_led_tick = 0;
    digitalWrite(LED_green, LOW);
  }
}
