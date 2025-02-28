#include <FlexCAN_T4.h>
#include <Arduino.h>
#include <Bounce.h>
#include "ecu_sim.h"

/*******************************************************
 * Global variables and definitions
 *******************************************************/
extern uint16_t flash_led_tick;  // If used for LED blinking logic


// Instantiate the simulator class
ecu_simClass ecu_sim;

// Debounced push buttons
Bounce pushbuttonSW1 = Bounce(SW1, 10);
Bounce pushbuttonSW2 = Bounce(SW2, 10);

// Create a FlexCAN_T4 instance at global scope.
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> CANbus;

/*******************************************************
 * Simulated strings for Mode 9 (VIN, CALID, CVN)
 *******************************************************/
static const char simulated_vin[]    = "3TMCZ5ANXJM137018"; 
static const char simulated_calid[] = "000007615981";
static const char simulated_cvn[]   = "B34322E5";

/*******************************************************
 * Forward declarations for Mode 9 helper functions
 *******************************************************/
static void respondWithVIN();
static void respondWithCALID();
static void respondWithCVN();
static void sendNegativeResponse(uint8_t service, uint8_t pid);
static bool waitForFlowControl();
static void printFrameData(const CAN_message_t &msg);
static String hexByte(uint8_t b);

/*******************************************************
 * ecu_simClass Implementation
 *******************************************************/
ecu_simClass::ecu_simClass() {
}

uint8_t ecu_simClass::init(uint32_t baud) {
  pinMode(SW1, INPUT_PULLUP);
  pinMode(SW2, INPUT_PULLUP);

  // LED pins for "check engine" (red) and "activity" (green)
  pinMode(LED_red,   OUTPUT);
  pinMode(LED_green, OUTPUT);
  digitalWrite(LED_red,   LOW);
  digitalWrite(LED_green, LOW);

  // Initialize CAN bus at specified baud (e.g., 500000)
  CANbus.begin();
  CANbus.setBaudRate(baud);
  CANbus.setMBFilter(ACCEPT_ALL);
  CANbus.distribute();
  CANbus.mailboxStatus();

  // By default, no trouble codes set
  ecu.dtc = 0;

  // Initialize our new fields
  ecu.engine_load      = 0;
  ecu.intake_air_temp  = 0;

  return 0;
}

/**
 * Update the simulator’s analog inputs (potentiometers) and
 * check whether the SW1 button was pressed to toggle DTC.
 */
void ecu_simClass::update_pots(void) {
  // Example usage of analog inputs for existing PIDs:
  ecu.engine_rpm        = 0xFFFF - map(analogRead(AN1), 0, 1023, 0, 0xFFFF);
  ecu.vehicle_speed     = 0xFF   - map(analogRead(AN3), 0, 1023, 0, 0xFF);
  ecu.coolant_temp      = 0xFF   - map(analogRead(AN2), 0, 1023, 0, 0xFF);
  ecu.maf_airflow       = 0xFFFF - map(analogRead(AN4), 0, 1023, 0, 0xFFFF);
  ecu.throttle_position = 0xFF   - map(analogRead(AN5), 0, 1023, 0, 0xFF);
  ecu.o2_voltage        = 0xFFFF - map(analogRead(AN6), 0, 1023, 0, 0xFFFF);

  // NEW: Example for engine load (AN7 if available)
  // Range 0-255 => 0-100% load
  ecu.engine_load = map(analogRead(AN7), 0, 1023, 0, 255);

  // NEW: Example for intake air temp (could reuse AN6 or another pin)
  // Range 0-255 => (A - 40) °C in real OBD
  // We'll just store 0..255 in the struct and let the formula be done in the PID parse
  // For demonstration, reusing AN2 or a new pin. Let's reuse AN2 offset by 50 for variety
  // (Just an example. You can do whatever math you like.)
  uint16_t rawIAT = analogRead(AN2);
  ecu.intake_air_temp = map(rawIAT, 0, 1023, 0, 255);

  // Use SW1 to toggle a stored DTC
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

/**
 * Main update function to read CAN messages and respond
 * to Mode 1, 3, 4, 9 requests, etc.
 */
uint8_t ecu_simClass::update(void) {
  CAN_message_t can_MsgRx, can_MsgTx;

  // Check if a new CAN frame is available
  if (CANbus.readMB(can_MsgRx)) {
    Serial.print(can_MsgRx.id, HEX);
    Serial.print(" len:");
    Serial.print(can_MsgRx.len);
    Serial.print(" ");

    // Print the received bytes for debugging
    for (int i = 0; i < 8; i++) {
      Serial.print(can_MsgRx.buf[i], HEX);
      Serial.print(" ");
    }
    Serial.println();

    // Only process if this is a PID request (0x7DF)
    if (can_MsgRx.id == PID_REQUEST) {
      // Indicate activity
      digitalWrite(LED_green, HIGH);
      flash_led_tick = 0;

      // -------------------------------------------------
      // MODE 3: Request trouble codes
      // -------------------------------------------------
      if (can_MsgRx.buf[1] == MODE3) {
        if (ecu.dtc == 0) {
          // No stored DTC
          can_MsgTx.buf[0] = 0x02;
          can_MsgTx.buf[1] = MODE3_RESPONSE; // 0x43
          can_MsgTx.buf[2] = 0x00;
        } else {
          // Two stored codes
          can_MsgTx.buf[0] = 0x06;
          can_MsgTx.buf[1] = MODE3_RESPONSE; // 0x43
          can_MsgTx.buf[2] = 0x02;
          can_MsgTx.buf[3] = 0x01;
          can_MsgTx.buf[4] = 0x00;
          can_MsgTx.buf[5] = 0x02;
          can_MsgTx.buf[6] = 0x00;
        }
        can_MsgTx.id  = PID_REPLY;  // 0x7E8
        can_MsgTx.len = 8;
        CANbus.write(can_MsgTx);
      }

      // -------------------------------------------------
      // MODE 4: Clear trouble codes
      // -------------------------------------------------
      else if (can_MsgRx.buf[1] == MODE4) {
        ecu.dtc = 0;
        digitalWrite(LED_red, LOW);

        can_MsgTx.buf[0] = 0x00;
        can_MsgTx.buf[1] = MODE4_RESPONSE; // 0x44
        for (int i = 2; i < 8; i++) {
          can_MsgTx.buf[i] = 0x00;
        }
        can_MsgTx.id  = PID_REPLY;
        can_MsgTx.len = 8;
        CANbus.write(can_MsgTx);
      }

      // -------------------------------------------------
      // MODE 1: Normal PIDs (RPM, COOLANT, etc.)
      // -------------------------------------------------
      else if (can_MsgRx.buf[1] == MODE1) {
        can_MsgTx.id  = PID_REPLY;
        can_MsgTx.len = 8;
        can_MsgTx.buf[1] = MODE1_RESPONSE; // 0x41

        switch (can_MsgRx.buf[2]) {
          case PID_SUPPORTED:
            can_MsgTx.buf[0] = 0x06;
            can_MsgTx.buf[2] = PID_SUPPORTED; 
            can_MsgTx.buf[3] = 0xE8; // example bits for 0100–0120
            can_MsgTx.buf[4] = 0x19;
            can_MsgTx.buf[5] = 0x30;
            can_MsgTx.buf[6] = 0x12;
            CANbus.write(can_MsgTx);
            break;

          case MONITOR_STATUS:
            can_MsgTx.buf[0] = 0x05;
            can_MsgTx.buf[2] = MONITOR_STATUS;
            can_MsgTx.buf[3] = (ecu.dtc == 1) ? 0x82 : 0x00;
            can_MsgTx.buf[4] = 0x07;
            can_MsgTx.buf[5] = 0xFF;
            CANbus.write(can_MsgTx);
            break;

          case ENGINE_RPM:
            //   ((A*256)+B)/4 [RPM]
            can_MsgTx.buf[0] = 0x04;
            can_MsgTx.buf[2] = ENGINE_RPM;
            can_MsgTx.buf[3] = (ecu.engine_rpm & 0xff00) >> 8;
            can_MsgTx.buf[4] = ecu.engine_rpm & 0x00ff;
            CANbus.write(can_MsgTx);
            break;

          case ENGINE_COOLANT_TEMP:
            //   A - 40 [°C]
            can_MsgTx.buf[0] = 0x03;
            can_MsgTx.buf[2] = ENGINE_COOLANT_TEMP;
            can_MsgTx.buf[3] = ecu.coolant_temp;
            CANbus.write(can_MsgTx);
            break;

          case VEHICLE_SPEED:
            //   A [km/h]
            can_MsgTx.buf[0] = 0x03;
            can_MsgTx.buf[2] = VEHICLE_SPEED;
            can_MsgTx.buf[3] = ecu.vehicle_speed;
            CANbus.write(can_MsgTx);
            break;

          case MAF_SENSOR:
            //   ((256*A)+B)/100 [g/s]
            can_MsgTx.buf[0] = 0x04;
            can_MsgTx.buf[2] = MAF_SENSOR;
            can_MsgTx.buf[3] = (ecu.maf_airflow & 0xff00) >> 8;
            can_MsgTx.buf[4] = ecu.maf_airflow & 0x00ff;
            CANbus.write(can_MsgTx);
            break;

          case O2_VOLTAGE:
            //   A * 0.005 [V], (B-128) * 100/128
            can_MsgTx.buf[0] = 0x04;
            can_MsgTx.buf[2] = O2_VOLTAGE;
            can_MsgTx.buf[3] = (ecu.o2_voltage & 0x00ff);
            can_MsgTx.buf[4] = (ecu.o2_voltage & 0xff00) >> 8;
            CANbus.write(can_MsgTx);
            break;

          case THROTTLE:
            //   A * 100/255 [%]
            can_MsgTx.buf[0] = 0x03;
            can_MsgTx.buf[2] = THROTTLE;
            can_MsgTx.buf[3] = ecu.throttle_position;
            CANbus.write(can_MsgTx);

            Serial.print("Throttle: ");
            Serial.println(ecu.throttle_position, HEX);
            break;

          // NEW CASES
          case ENGINE_LOAD:
            //   A * 100/255 [%]
            can_MsgTx.buf[0] = 0x03;
            can_MsgTx.buf[2] = ENGINE_LOAD; 
            can_MsgTx.buf[3] = ecu.engine_load; 
            CANbus.write(can_MsgTx);
            break;

          case INTAKE_AIR_TEMP:
            //   A - 40 [°C]
            can_MsgTx.buf[0] = 0x03;
            can_MsgTx.buf[2] = INTAKE_AIR_TEMP;
            can_MsgTx.buf[3] = ecu.intake_air_temp;
            CANbus.write(can_MsgTx);
            break;

          default:
            // If we got an unrecognized PID, you can respond with "NO DATA"
            // or send negative response, etc.
            sendNegativeResponse(can_MsgRx.buf[1], can_MsgRx.buf[2]);
            break;
        }
      }

      // -------------------------------------------------
      // MODE 9: Request Vehicle Information (VIN, CALID, CVN)
      // -------------------------------------------------
      else if (can_MsgRx.buf[1] == MODE9) {
        switch (can_MsgRx.buf[2]) {
          case VIN_PID:
            respondWithVIN();
            break;
          case CALIBRATION_ID:
            respondWithCALID();
            break;
          case CALIBRATION_VERIFICATION:
            respondWithCVN();
            break;
          default:
            // Unknown PID -> send negative response
            sendNegativeResponse(can_MsgRx.buf[1], can_MsgRx.buf[2]);
            break;
        }
      }
    }
  }
  return 0;
}

/*******************************************************
 * Mode 9 Helper Implementations
 *******************************************************/
static void respondWithVIN() {
  uint8_t totalPayloadLen = 3 + 17; // 3 bytes + 17 VIN chars
  uint8_t count           = 0x01;   

  CAN_message_t txMsg;
  txMsg.id  = PID_REPLY;  // 0x7E8
  txMsg.len = 8;

  // First Frame
  txMsg.buf[0] = 0x10 | ((totalPayloadLen >> 8) & 0x0F);
  txMsg.buf[1] = totalPayloadLen & 0xFF;
  txMsg.buf[2] = 0x49;    
  txMsg.buf[3] = VIN_PID; 
  txMsg.buf[4] = count;   
  txMsg.buf[5] = simulated_vin[0];
  txMsg.buf[6] = simulated_vin[1];
  txMsg.buf[7] = simulated_vin[2];
  CANbus.write(txMsg);
  Serial.print("Sent First Frame for VIN: ");
  printFrameData(txMsg);

  if (!waitForFlowControl()) {
    Serial.println("No Flow Control received for VIN, aborting.");
    return;
  }

  // Consecutive frames
  uint8_t seq      = 1;
  uint8_t vinIndex = 3;
  uint8_t remaining = 17 - vinIndex;
  while (remaining > 0) {
    txMsg.id  = PID_REPLY;
    txMsg.len = 8;
    txMsg.buf[0] = 0x20 | (seq & 0x0F);
    uint8_t bytesThisFrame = (remaining > 7 ? 7 : remaining);
    for (uint8_t i = 0; i < bytesThisFrame; i++) {
      txMsg.buf[1 + i] = simulated_vin[vinIndex + i];
    }
    for (uint8_t i = bytesThisFrame; i < 7; i++) {
      txMsg.buf[1 + i] = 0xCC;
    }
    CANbus.write(txMsg);

    Serial.print("Sent CF ");
    Serial.print(seq);
    Serial.print(" for VIN: ");
    printFrameData(txMsg);

    seq = (seq + 1) & 0x0F;
    vinIndex  += bytesThisFrame;
    remaining -= bytesThisFrame;
  }
}

static void respondWithCALID() {
  uint8_t calidLen     = strlen(simulated_calid);
  uint8_t totalPayload = 3 + calidLen;
  uint8_t count        = 0x01;

  CAN_message_t tx;
  tx.id  = PID_REPLY;
  tx.len = 8;

  // Single-frame or multi-frame
  if (totalPayload <= 7) {
    tx.buf[0] = totalPayload;
    tx.buf[1] = 0x49;
    tx.buf[2] = CALIBRATION_ID;
    tx.buf[3] = count;
    for (uint8_t i = 0; i < calidLen; i++) {
      tx.buf[4 + i] = simulated_calid[i];
    }
    for (uint8_t i = (4 + calidLen); i < 8; i++) {
      tx.buf[i] = 0xCC;
    }
    CANbus.write(tx);
    Serial.print("Sent Single Frame CALID: ");
    printFrameData(tx);
  } else {
    tx.buf[0] = 0x10 | ((totalPayload >> 8) & 0x0F);
    tx.buf[1] = totalPayload & 0xFF;
    tx.buf[2] = 0x49;
    tx.buf[3] = CALIBRATION_ID;
    tx.buf[4] = count;
    tx.buf[5] = (calidLen > 0) ? simulated_calid[0] : 0;
    tx.buf[6] = (calidLen > 1) ? simulated_calid[1] : 0;
    tx.buf[7] = (calidLen > 2) ? simulated_calid[2] : 0;
    uint8_t calIndex = 3;

    CANbus.write(tx);
    Serial.print("Sent First Frame for CALID: ");
    printFrameData(tx);

    if (!waitForFlowControl()) {
      Serial.println("No Flow Control for CALID, aborting.");
      return;
    }

    uint8_t seq = 1;
    uint8_t remaining = calidLen - calIndex;
    while (remaining > 0) {
      tx.id  = PID_REPLY;
      tx.len = 8;
      tx.buf[0] = 0x20 | (seq & 0x0F);
      uint8_t bytesThisFrame = (remaining > 7 ? 7 : remaining);
      for (uint8_t i = 0; i < bytesThisFrame; i++) {
        tx.buf[1 + i] = simulated_calid[calIndex + i];
      }
      for (uint8_t i = bytesThisFrame; i < 7; i++) {
        tx.buf[1 + i] = 0xCC;
      }
      CANbus.write(tx);

      Serial.print("Sent CF ");
      Serial.print(seq);
      Serial.print(" for CALID: ");
      printFrameData(tx);

      seq = (seq + 1) & 0x0F;
      calIndex  += bytesThisFrame;
      remaining -= bytesThisFrame;
    }
  }
}

static void respondWithCVN() {
  uint8_t cvnLen       = strlen(simulated_cvn);
  uint8_t totalPayload = 3 + cvnLen;
  uint8_t count        = 0x01;

  CAN_message_t tx;
  tx.id  = PID_REPLY;
  tx.len = 8;

  if (totalPayload <= 7) {
    tx.buf[0] = totalPayload;
    tx.buf[1] = 0x49;
    tx.buf[2] = CALIBRATION_VERIFICATION;
    tx.buf[3] = count;
    for (uint8_t i = 0; i < cvnLen; i++) {
      tx.buf[4 + i] = simulated_cvn[i];
    }
    for (uint8_t i = (4 + cvnLen); i < 8; i++) {
      tx.buf[i] = 0xCC;
    }
    CANbus.write(tx);
    Serial.print("Sent Single Frame CVN: ");
    printFrameData(tx);
  } else {
    tx.buf[0] = 0x10 | ((totalPayload >> 8) & 0x0F);
    tx.buf[1] = totalPayload & 0xFF;
    tx.buf[2] = 0x49;
    tx.buf[3] = CALIBRATION_VERIFICATION;
    tx.buf[4] = count;
    tx.buf[5] = (cvnLen > 0) ? simulated_cvn[0] : 0;
    tx.buf[6] = (cvnLen > 1) ? simulated_cvn[1] : 0;
    tx.buf[7] = (cvnLen > 2) ? simulated_cvn[2] : 0;
    uint8_t cvnIndex = 3;

    CANbus.write(tx);
    Serial.print("Sent First Frame for CVN: ");
    printFrameData(tx);

    if (!waitForFlowControl()) {
      Serial.println("No Flow Control for CVN, aborting.");
      return;
    }

    uint8_t seq = 1;
    uint8_t remaining = cvnLen - cvnIndex;
    while (remaining > 0) {
      tx.id  = PID_REPLY;
      tx.len = 8;
      tx.buf[0] = 0x20 | (seq & 0x0F);
      uint8_t bytesThisFrame = (remaining > 7 ? 7 : remaining);
      for (uint8_t i = 0; i < bytesThisFrame; i++) {
        tx.buf[1 + i] = simulated_cvn[cvnIndex + i];
      }
      for (uint8_t i = bytesThisFrame; i < 7; i++) {
        tx.buf[1 + i] = 0xCC;
      }
      CANbus.write(tx);

      Serial.print("Sent CF ");
      Serial.print(seq);
      Serial.print(" for CVN: ");
      printFrameData(tx);

      seq = (seq + 1) & 0x0F;
      cvnIndex  += bytesThisFrame;
      remaining -= bytesThisFrame;
    }
  }
}

static void sendNegativeResponse(uint8_t service, uint8_t pid) {
  CAN_message_t tx;
  tx.id  = PID_REPLY;
  tx.len = 8;

  tx.buf[0] = 0x03; // 3 data bytes
  tx.buf[1] = 0x7F; // Negative response
  tx.buf[2] = service;
  tx.buf[3] = 0x12; // e.g., "SubFunction Not Supported"
  for (uint8_t i = 4; i < 8; i++) {
    tx.buf[i] = 0xCC;
  }
  CANbus.write(tx);

  Serial.print("Sent Negative Response for service 0x");
  Serial.print(service, HEX);
  Serial.print(", PID 0x");
  Serial.println(pid, HEX);
}

static bool waitForFlowControl() {
  unsigned long start = millis();
  CAN_message_t rx;

  while (millis() - start < 100) {
    if (CANbus.read(rx)) {
      if (rx.id == ECU_PHYS_ID) { // 0x7E0
        uint8_t pci = rx.buf[0];
        if ((pci & 0xF0) == 0x30) {
          uint8_t flowStatus = pci & 0x0F;
          if (flowStatus == 0) {
            Serial.println("Flow Control: Continue");
            return true;
          } else if (flowStatus == 1) {
            Serial.println("Flow Control: Wait");
            delay(50);
            return true;
          } else {
            Serial.println("Flow Control: Abort");
            return false;
          }
        }
      }
    }
  }
  return false;
}

static void printFrameData(const CAN_message_t &msg) {
  Serial.print("[ ");
  for (uint8_t i = 0; i < msg.len; i++) {
    Serial.print(hexByte(msg.buf[i]));
    Serial.print(" ");
  }
  Serial.println("]");
}

static String hexByte(uint8_t b) {
  char buf[3];
  snprintf(buf, sizeof(buf), "%02X", b);
  return String(buf);
}
