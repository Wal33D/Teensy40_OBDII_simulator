/*
  Teensy 4.0 OBDII CAN-BUS ECU simulator.
  www.skpang.co.uk
  December 2022

  For use with:
  https://www.skpang.co.uk/collections/teensy/products/teensy-4-0-obdii-can-bus-ecu-simulator-with-teensy-4-0

  Other PIDs can be added, more info on PID:
  http://en.wikipedia.org/wiki/OBD-II_PIDs
*/

#include <Arduino.h>
#include "ecu_sim.h"

IntervalTimer timer;

ecu_t ecu;
uint16_t led_tick = 0;
uint16_t pot_tick = 0;
uint16_t flash_led_tick = 0;

int led = 13;   // On-board LED on Teensy

void tick(void)
{
    led_tick++;
    pot_tick++;
    flash_led_tick++;
}

void setup() {
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

  Serial.begin(115200);
  Serial.println("****** Teensy 4.0 OBDII simulator skpang.co.uk 2022 ******");

  // Initialize CAN simulator code
  ecu_sim.init(500000);

  // Start a 1ms interrupt timer
  timer.begin(tick, 1000);    // 1ms = 1000 microseconds
}

void loop() {

  // Process incoming CAN messages and respond
  ecu_sim.update();

  // Blink on-board LED every ~1 second
  if (led_tick > 1000) {
    led_tick = 0;
    digitalToggle(led);
  }

  // Update pot values ~every 10ms
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
