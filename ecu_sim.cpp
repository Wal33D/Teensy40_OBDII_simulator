/*
   CAN Bus ECU Simulator
   www.skpang.co.uk
*/

#include "ecu_sim.h"
#include <Bounce.h>
#include <FlexCAN_T4.h>

// Extern variable used in main .ino for LED timing
extern uint16_t flash_led_tick;

// Example 17-character VIN to return
// (Change this to match what you want to simulate.)
static const char VIN[] = "1HGCM82633A123456";

Bounce pushbuttonSW1 = Bounce(SW1, 10);
Bounce pushbuttonSW2 = Bounce(SW2, 10);

// Setup CAN on Teensy 4.0
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

ecu_simClass::ecu_simClass() {
  // Constructor
}

uint8_t ecu_simClass::init(uint32_t baud) {
  pinMode(SW1, INPUT_PULLUP);
  pinMode(SW2, INPUT_PULLUP);

  can1.begin();
  can1.setBaudRate(baud);
  can1.setMBFilter(ACCEPT_ALL);
  can1.distribute();
  can1.mailboxStatus();

  ecu.dtc = 0;
  return 0;
}

void ecu_simClass::update_pots(void) {
  // Read pots and map them to certain ECU parameters
  ecu.engine_rpm        = 0xFFFF - map(analogRead(AN1), 0, 1023, 0, 0xFFFF);
  ecu.vehicle_speed     = 0xFF   - map(analogRead(AN3), 0, 1023, 0, 0xFF);
  ecu.coolant_temp      = 0xFF   - map(analogRead(AN2), 0, 1023, 0, 0xFF);
  ecu.maf_airflow       = 0xFFFF - map(analogRead(AN4), 0, 1023, 0, 0xFFFF);
  ecu.throttle_position = 0xFF   - map(analogRead(AN5), 0, 1023, 0, 0xFF);
  ecu.o2_voltage        = 0xFFFF - map(analogRead(AN6), 0, 1023, 0, 0xFFFF);

  // Check button SW1 for toggling a DTC
  if (pushbuttonSW1.update()) {
    if (pushbuttonSW1.fallingEdge()) {
      if (ecu.dtc == 0) {
        ecu.dtc = 1;
        digitalWrite(LED_red, HIGH);
      } else {
        ecu.dtc = 0;
        digitalWrite(LED_red, LOW);
      }
    }
  }
}

uint8_t ecu_simClass::update(void) {
  CAN_message_t can_MsgRx, can_MsgTx;

  if (can1.readMB(can_MsgRx)) {
    // Optional debug printing:
    // Serial.print(can_MsgRx.id,HEX); Serial.print(" len:");
    // Serial.print(can_MsgRx.len); Serial.print(" data: ");
    // for (int i=0; i<can_MsgRx.len; i++) {
    //   Serial.print(can_MsgRx.buf[i], HEX); Serial.print(" ");
    // }
    // Serial.println();

    // Listen for requests with ID = 0x7DF (PID_REQUEST)
    if (can_MsgRx.id == PID_REQUEST) {
      // Light the green LED briefly
      digitalWrite(LED_green, HIGH);
      flash_led_tick = 0;

      // --- MODE 3: Request Trouble Codes ---
      if (can_MsgRx.buf[1] == MODE3) {
        can_MsgTx.id  = PID_REPLY;   // 7E8
        can_MsgTx.len = 8;

        if (ecu.dtc == 0) {
          // No DTC
          can_MsgTx.buf[0] = 0x02;
          can_MsgTx.buf[1] = MODE3_RESPONSE;  // 0x43
          can_MsgTx.buf[2] = 0x00;  
          // Rest can be 0
          can_MsgTx.buf[3] = 0x00;
          can_MsgTx.buf[4] = 0x00;
          can_MsgTx.buf[5] = 0x00;
          can_MsgTx.buf[6] = 0x00;
          can_MsgTx.buf[7] = 0x00;
        } else {
          // Example DTC data (2 trouble codes, etc.)
          can_MsgTx.buf[0] = 0x06;
          can_MsgTx.buf[1] = MODE3_RESPONSE;   // 0x43
          can_MsgTx.buf[2] = 0x02;            // Number of DTCs
          can_MsgTx.buf[3] = 0x01;
          can_MsgTx.buf[4] = 0x00;
          can_MsgTx.buf[5] = 0x02;
          can_MsgTx.buf[6] = 0x00;
          can_MsgTx.buf[7] = 0x00;
        }
        can1.write(can_MsgTx);
      }

      // --- MODE 4: Clear Trouble Codes ---
      if (can_MsgRx.buf[1] == MODE4) {
        ecu.dtc = 0;
        digitalWrite(LED_red, LOW);

        can_MsgTx.id  = PID_REPLY;          // 7E8
        can_MsgTx.len = 8;
        can_MsgTx.buf[0] = 0x00;
        can_MsgTx.buf[1] = MODE4_RESPONSE;  // 0x44
        // Pad out with zeros
        for (int i=2; i<8; i++) can_MsgTx.buf[i] = 0x00;
        can1.write(can_MsgTx);
      }

      // --- MODE 1: Show Current Data ---
      if (can_MsgRx.buf[1] == MODE1) {
        can_MsgTx.id  = PID_REPLY;    // 7E8
        can_MsgTx.len = 8;
        can_MsgTx.buf[1] = MODE1_RESPONSE;  // 0x41

        switch (can_MsgRx.buf[2]) {
          case PID_SUPPORTED:
            can_MsgTx.buf[0] = 0x06;  
            can_MsgTx.buf[2] = PID_SUPPORTED; 
            can_MsgTx.buf[3] = 0xE8;
            can_MsgTx.buf[4] = 0x19;
            can_MsgTx.buf[5] = 0x30;
            can_MsgTx.buf[6] = 0x12;
            can_MsgTx.buf[7] = 0x00;
            can1.write(can_MsgTx);
            break;

          case MONITOR_STATUS:
            can_MsgTx.buf[0] = 0x05;  
            can_MsgTx.buf[2] = MONITOR_STATUS; 
            can_MsgTx.buf[3] = (ecu.dtc == 1) ? 0x82 : 0x00;
            can_MsgTx.buf[4] = 0x07;
            can_MsgTx.buf[5] = 0xFF;
            can_MsgTx.buf[6] = 0x00;
            can_MsgTx.buf[7] = 0x00;
            can1.write(can_MsgTx);
            break;

          case ENGINE_RPM:  // ((A*256)+B)/4
            can_MsgTx.buf[0] = 0x04;  
            can_MsgTx.buf[2] = ENGINE_RPM; 
            can_MsgTx.buf[3] = (ecu.engine_rpm >> 8) & 0xFF;
            can_MsgTx.buf[4] = ecu.engine_rpm & 0xFF;
            can_MsgTx.buf[5] = 0x00;
            can_MsgTx.buf[6] = 0x00;
            can_MsgTx.buf[7] = 0x00;
            can1.write(can_MsgTx);
            break;

          case ENGINE_COOLANT_TEMP: // A - 40
            can_MsgTx.buf[0] = 0x03;  
            can_MsgTx.buf[2] = ENGINE_COOLANT_TEMP; 
            can_MsgTx.buf[3] = ecu.coolant_temp;
            can_MsgTx.buf[4] = 0x00;
            can_MsgTx.buf[5] = 0x00;
            can_MsgTx.buf[6] = 0x00;
            can_MsgTx.buf[7] = 0x00;
            can1.write(can_MsgTx);
            break;

          case VEHICLE_SPEED:  // A
            can_MsgTx.buf[0] = 0x03;  
            can_MsgTx.buf[2] = VEHICLE_SPEED; 
            can_MsgTx.buf[3] = ecu.vehicle_speed;
            can_MsgTx.buf[4] = 0x00;
            can_MsgTx.buf[5] = 0x00;
            can_MsgTx.buf[6] = 0x00;
            can_MsgTx.buf[7] = 0x00;
            can1.write(can_MsgTx);
            break;

          case MAF_SENSOR: // ((256*A)+B)/100
            can_MsgTx.buf[0] = 0x04;  
            can_MsgTx.buf[2] = MAF_SENSOR; 
            can_MsgTx.buf[3] = (ecu.maf_airflow >> 8) & 0xFF;
            can_MsgTx.buf[4] = ecu.maf_airflow & 0xFF;
            can_MsgTx.buf[5] = 0x00;
            can_MsgTx.buf[6] = 0x00;
            can_MsgTx.buf[7] = 0x00;
            can1.write(can_MsgTx);
            break;

          case O2_VOLTAGE:
            can_MsgTx.buf[0] = 0x04;  
            can_MsgTx.buf[2] = O2_VOLTAGE; 
            can_MsgTx.buf[3] = ecu.o2_voltage & 0xFF;
            can_MsgTx.buf[4] = (ecu.o2_voltage >> 8) & 0xFF;
            can_MsgTx.buf[5] = 0x00;
            can_MsgTx.buf[6] = 0x00;
            can_MsgTx.buf[7] = 0x00;
            can1.write(can_MsgTx);
            break;

          case THROTTLE:
            can_MsgTx.buf[0] = 0x03;  
            can_MsgTx.buf[2] = THROTTLE; 
            can_MsgTx.buf[3] = ecu.throttle_position;
            can_MsgTx.buf[4] = 0x00;
            can_MsgTx.buf[5] = 0x00;
            can_MsgTx.buf[6] = 0x00;
            can_MsgTx.buf[7] = 0x00;
            can1.write(can_MsgTx);
            // Serial.print("Throttle: ");
            // Serial.println(ecu.throttle_position, HEX);
            break;
        } // end switch
      } // end if Mode1

      // --- MODE 9: Vehicle Information (VIN) ---
      if (can_MsgRx.buf[1] == MODE9) {
        // Return VIN on PID=0x02
        if (can_MsgRx.buf[2] == VIN_PID) {
          // We'll do a naive multi-frame response (3 frames) for 17 bytes of VIN

          // 1) First Frame
          // Byte[0] = 0x10 | (high nibble of total length if > 15); total = 17
          // Byte[1] = low byte of total length
          can_MsgTx.id  = PID_REPLY;  // 7E8
          can_MsgTx.len = 8;
          // Indicate First Frame: upper nibble=1 (for FF), lower nibble=0
          can_MsgTx.buf[0] = 0x10; 
          // Next byte is total length of data (17)
          can_MsgTx.buf[1] = 17;
          // Then Mode 9 response (0x49) + PID (0x02)
          can_MsgTx.buf[2] = MODE9_RESPONSE; // 0x49
          can_MsgTx.buf[3] = VIN_PID;        // 0x02
          // Put first 4 chars of VIN in the remainder
          can_MsgTx.buf[4] = VIN[0];
          can_MsgTx.buf[5] = VIN[1];
          can_MsgTx.buf[6] = VIN[2];
          can_MsgTx.buf[7] = VIN[3];
          can1.write(can_MsgTx);

          delayMicroseconds(500); // Short delay, ignoring flow control

          // 2) Consecutive Frame #1
          can_MsgTx.buf[0] = 0x21;  // CF sequence #1
          can_MsgTx.buf[1] = VIN[4];
          can_MsgTx.buf[2] = VIN[5];
          can_MsgTx.buf[3] = VIN[6];
          can_MsgTx.buf[4] = VIN[7];
          can_MsgTx.buf[5] = VIN[8];
          can_MsgTx.buf[6] = VIN[9];
          can_MsgTx.buf[7] = VIN[10];
          can1.write(can_MsgTx);

          delayMicroseconds(500);

          // 3) Consecutive Frame #2
          can_MsgTx.buf[0] = 0x22;  // CF sequence #2
          can_MsgTx.buf[1] = VIN[11];
          can_MsgTx.buf[2] = VIN[12];
          can_MsgTx.buf[3] = VIN[13];
          can_MsgTx.buf[4] = VIN[14];
          can_MsgTx.buf[5] = VIN[15];
          can_MsgTx.buf[6] = VIN[16];
          // Byte[7] can be padding
          can_MsgTx.buf[7] = 0x00;
          can1.write(can_MsgTx);
        }
      } // end if Mode9
    } // end if (PID_REQUEST)
  } // end if (can1.readMB(can_MsgRx))

  return 0;
}

// Instantiate the global ecu_sim object
ecu_simClass ecu_sim;
