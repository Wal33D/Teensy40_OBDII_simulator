#include "Print.h"
/*
 * Advanced OBD-II CAN Bus ECU Simulator
 *
 * Original: www.skpang.co.uk
 * Enhanced by: Waleed Judah (Wal33D)
 *
 * This simulator implements the emissions monitoring aspects of OBD-II.
 * OBD-II is primarily an emissions control program mandated by EPA/CARB,
 * not a general diagnostic system. All data provided relates to emissions
 * monitoring and control functions.
 */

#include <Bounce.h>
#include "ecu_sim.h"
#include <FlexCAN_T4.h>
#include "mode_registry.h"
#include "mode_includes.h"

Bounce pushbuttonSW1 = Bounce(SW1, 10);
Bounce pushbuttonSW2 = Bounce(SW2, 10);

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;  // can1 port 
extern uint16_t flash_led_tick;

ecu_simClass::ecu_simClass() {
 
}

uint8_t ecu_simClass::init(uint32_t baud) {
  pinMode(SW1,INPUT_PULLUP);
  pinMode(SW2,INPUT_PULLUP);
  can1.begin();
  can1.setBaudRate(500000);
  can1.setMBFilter(ACCEPT_ALL);
  can1.distribute();
  can1.mailboxStatus();

  ecu.dtc = 0;  // No emissions DTCs stored

  // Initialize Mode 02 freeze frame storage
  // Required by OBD-II to help diagnose intermittent emissions faults
  freeze_frame[0].data_stored = false;
  freeze_frame[1].data_stored = false;

  // Initialize with realistic idle values from real Mercedes-Benz
  // These represent a warmed-up engine meeting emissions standards
  ecu.coolant_temp = 95 + 40;  // 95°C - optimal for catalytic converter
  ecu.engine_rpm = 614 * 4;    // 614 RPM - typical idle for emissions
  ecu.vehicle_speed = 0;        // 0 km/h - stationary
  ecu.throttle_position = 30;   // 11.8% - idle throttle for emissions
  ecu.maf_airflow = 0;          // Will be calculated dynamically
  ecu.o2_voltage = 0x3C;        // 0.3V - indicates proper combustion

  return 0;
}
void ecu_simClass::update_pots(void) 
{
  uint16_t temp;
  ecu.engine_rpm = 0xffff - map(analogRead(AN1), 0, 1023, 0, 0xffff);
  ecu.vehicle_speed = 0xff - map(analogRead(AN3), 0, 1023, 0, 0xff);
  ecu.coolant_temp =  0xff - map(analogRead(AN2), 0, 1023, 0, 0xff);
  ecu.maf_airflow = 0xffff - map(analogRead(AN4), 0, 1023, 0, 0xffff);
  ecu.throttle_position = 0xff - map(analogRead(AN5), 0, 1023, 0, 0xff);
  ecu.o2_voltage = 0xffff - map(analogRead(AN6), 0, 1023, 0, 0xffff);
 
  if (pushbuttonSW1.update()) 
  {
    if (pushbuttonSW1.fallingEdge()) 
    {
      if(ecu.dtc == 0)
      {
          ecu.dtc = 1;
          digitalWrite(LED_red, HIGH);

          // Capture freeze frame data when DTC is triggered
          freeze_frame[0].coolant_temp = ecu.coolant_temp;
          freeze_frame[0].engine_rpm = ecu.engine_rpm;
          freeze_frame[0].throttle_position = ecu.throttle_position;
          freeze_frame[0].vehicle_speed = ecu.vehicle_speed;
          freeze_frame[0].maf_airflow = ecu.maf_airflow;
          freeze_frame[0].o2_voltage = ecu.o2_voltage;
          freeze_frame[0].data_stored = true;
          freeze_frame[0].dtc_code = 0x0100;  // P0100

          freeze_frame[1].coolant_temp = ecu.coolant_temp;
          freeze_frame[1].engine_rpm = ecu.engine_rpm;
          freeze_frame[1].throttle_position = ecu.throttle_position;
          freeze_frame[1].vehicle_speed = ecu.vehicle_speed;
          freeze_frame[1].maf_airflow = ecu.maf_airflow;
          freeze_frame[1].o2_voltage = ecu.o2_voltage;
          freeze_frame[1].data_stored = true;
          freeze_frame[1].dtc_code = 0x0200;  // P0200

      }else 
      {
          ecu.dtc = 0;
          digitalWrite(LED_red, LOW);
      }
    }
  }
}


uint8_t ecu_simClass::update(void)
{
  CAN_message_t can_MsgRx,can_MsgTx;

  // Process any ongoing ISO-TP transfers
  isotp_process_transfers();

  if(can1.readMB(can_MsgRx))
  {
     Serial.print(can_MsgRx.id,HEX);Serial.print(" len:");
     Serial.print(can_MsgRx.len);Serial.print(" ");
     Serial.print(can_MsgRx.buf[0]);Serial.print(" ");
     Serial.print(can_MsgRx.buf[1]);Serial.print(" ");
     Serial.print(can_MsgRx.buf[2]);Serial.print(" ");
     Serial.print(can_MsgRx.buf[3]);Serial.print(" ");
     Serial.print(can_MsgRx.buf[4]);Serial.print(" ");
     Serial.print(can_MsgRx.buf[5]);Serial.print(" ");
     Serial.print(can_MsgRx.buf[6]);Serial.print(" ");
     Serial.print(can_MsgRx.buf[7]);Serial.println(" ");
     
     // Handle ISO-TP Flow Control frames from tester
     // Tester sends flow control on 0x7E0 (ECM), 0x7E1 (TCM), 0x7E3 (FPCM)
     if ((can_MsgRx.id >= 0x7E0 && can_MsgRx.id <= 0x7E7) &&
         (can_MsgRx.buf[0] & 0xF0) == ISO_TP_FLOW_CONTROL) {
         // Flow control received - pass to ISO-TP handler
         isotp_handle_flow_control(can_MsgRx.buf);
         return 0;  // Flow control processed
     }

     // Handle broadcast (0x7DF) and all ECU-specific requests
     // Supports 3 ECUs: ECM (0x7E0), TCM (0x7E1), FPCM (0x7E3)
     if (can_MsgRx.id == PID_REQUEST ||
         can_MsgRx.id == PID_REQUEST_ENGINE ||
         can_MsgRx.id == PID_REQUEST_TRANS ||
         can_MsgRx.id == 0x7E3)  // FPCM request
     {
       digitalWrite(LED_green, HIGH);
       flash_led_tick = 0;

       // Check if this is an ISO-TP First Frame from tester (for receiving multi-frame requests)
       if ((can_MsgRx.buf[0] & 0xF0) == ISO_TP_FIRST_FRAME) {
           // Multi-frame request from tester - send flow control
           // Determine which ECU should respond based on request ID
           uint16_t response_id = PID_REPLY_ENGINE;  // Default to ECM
           if (can_MsgRx.id == PID_REQUEST_TRANS) {
               response_id = PID_REPLY_TRANS;  // TCM
           } else if (can_MsgRx.id == 0x7E3) {
               response_id = PID_REPLY_CHASSIS;  // FPCM
           }

           CAN_message_t flowControl;
           flowControl.id = response_id;
           flowControl.len = 8;
           flowControl.buf[0] = ISO_TP_FLOW_CONTROL | FC_CONTINUE;  // 0x30 - Continue to send
           flowControl.buf[1] = ISO_TP_BS;        // Block size (0 = send all)
           flowControl.buf[2] = ISO_TP_STMIN;     // Separation time (10ms)
           for(int i = 3; i < 8; i++) flowControl.buf[i] = 0x00;  // Padding
           can1.write(flowControl);

           // For now, we don't process multi-frame requests from tester
           // This would be needed for Mode 0x2E (DynamicallyDefineDataIdentifier) etc.
           return 0;
       }

       // Check if this is a Consecutive Frame from tester
       if ((can_MsgRx.buf[0] & 0xF0) == ISO_TP_CONSEC_FRAME) {
           // We're receiving consecutive frames from tester
           // For now, just acknowledge and ignore
           return 0;
       }

        // Dispatch to registered mode handlers
        ModeRegistry::dispatch(can_MsgRx, can_MsgTx, this);

       }
    }
   return 0;
}
     
freeze_frame_t freeze_frame[2];  // Global freeze frame storage
isotp_transfer_t isotp_tx;        // Global ISO-TP transmit context
pending_transfer_t pending_transfers[MAX_PENDING_TRANSFERS];  // Queue for multi-ECU responses
uint8_t pending_transfer_count = 0;
ecu_simClass ecu_sim;

// ISO-TP Implementation Functions
void ecu_simClass::isotp_init_transfer(uint8_t* data, uint16_t len, uint16_t can_id, uint8_t mode, uint8_t pid) {
    isotp_tx.state = ISOTP_IDLE;
    isotp_tx.total_len = len;
    isotp_tx.offset = 0;
    isotp_tx.seq_num = 1;
    isotp_tx.block_size = 0;
    isotp_tx.blocks_sent = 0;
    isotp_tx.st_min = 0;
    isotp_tx.response_id = can_id;
    isotp_tx.mode = mode;
    isotp_tx.pid = pid;
    memcpy(isotp_tx.data, data, len);
}

void ecu_simClass::isotp_send_first_frame(void) {
    CAN_message_t msg;
    msg.id = isotp_tx.response_id;
    msg.len = 8;
    msg.buf[0] = 0x10 | ((isotp_tx.total_len >> 8) & 0x0F);  // First frame with length high nibble
    msg.buf[1] = isotp_tx.total_len & 0xFF;                  // Length low byte

    // Copy first 6 bytes of data
    for(int i = 0; i < 6 && i < isotp_tx.total_len; i++) {
        msg.buf[i+2] = isotp_tx.data[i];
    }

    isotp_tx.offset = 6;  // We've sent 6 bytes
    isotp_tx.state = ISOTP_WAIT_FC;  // Wait for flow control
    isotp_tx.fc_wait_start = millis();

    can1.write(msg);
}

void ecu_simClass::isotp_handle_flow_control(uint8_t* data) {
    if(isotp_tx.state != ISOTP_WAIT_FC && isotp_tx.state != ISOTP_WAIT_NEXT_FC) {
        return;  // Not waiting for flow control
    }

    uint8_t fs = data[0] & 0x0F;  // Flow Status

    if(fs == 0) {  // Continue to send
        isotp_tx.block_size = data[1];  // 0 means send all remaining
        isotp_tx.st_min = data[2];      // Minimum separation time
        isotp_tx.blocks_sent = 0;
        isotp_tx.state = ISOTP_SENDING_CF;
        isotp_tx.last_frame_time = millis();
    } else if(fs == 1) {  // Wait
        isotp_tx.state = ISOTP_WAIT_FC;  // Keep waiting
    } else if(fs == 2) {  // Overflow/Abort
        isotp_tx.state = ISOTP_ERROR;
    }
}

void ecu_simClass::isotp_send_consecutive_frame(void) {
    if(isotp_tx.state != ISOTP_SENDING_CF || isotp_tx.offset >= isotp_tx.total_len) {
        return;
    }

    // Check timing requirement
    uint32_t now = millis();
    uint32_t elapsed = now - isotp_tx.last_frame_time;

    // STmin handling: 0x00-0x7F = 0-127ms, 0xF1-0xF9 = 100-900us (we'll treat as 1-9ms)
    uint32_t required_delay = isotp_tx.st_min;
    if(isotp_tx.st_min >= 0xF1 && isotp_tx.st_min <= 0xF9) {
        required_delay = (isotp_tx.st_min - 0xF0);  // 1-9ms for simplicity
    }

    if(elapsed < required_delay) {
        return;  // Not time yet
    }

    CAN_message_t msg;
    msg.id = isotp_tx.response_id;
    msg.len = 8;
    msg.buf[0] = 0x20 | (isotp_tx.seq_num & 0x0F);  // Consecutive frame

    // Copy up to 7 bytes of data
    int bytes_to_copy = min(7, isotp_tx.total_len - isotp_tx.offset);
    for(int i = 0; i < 7; i++) {
        if(i < bytes_to_copy) {
            msg.buf[i+1] = isotp_tx.data[isotp_tx.offset + i];
        } else {
            msg.buf[i+1] = 0x00;  // Padding
        }
    }

    isotp_tx.offset += bytes_to_copy;
    isotp_tx.seq_num = (isotp_tx.seq_num + 1) & 0x0F;
    isotp_tx.blocks_sent++;
    isotp_tx.last_frame_time = now;

    can1.write(msg);

    // Check if we've sent all data
    if(isotp_tx.offset >= isotp_tx.total_len) {
        isotp_tx.state = ISOTP_IDLE;  // Transfer complete
    }
    // Check if we need to wait for next flow control
    else if(isotp_tx.block_size > 0 && isotp_tx.blocks_sent >= isotp_tx.block_size) {
        isotp_tx.state = ISOTP_WAIT_NEXT_FC;
        isotp_tx.fc_wait_start = now;
    }
}

// Queue a transfer to be sent later (for multi-ECU responses)
bool ecu_simClass::isotp_queue_transfer(uint8_t* data, uint16_t len, uint16_t can_id, uint8_t mode, uint8_t pid) {
    // Find an empty slot in the queue
    for(int i = 0; i < MAX_PENDING_TRANSFERS; i++) {
        if(!pending_transfers[i].pending) {
            // Copy data to queue
            memcpy(pending_transfers[i].data, data, len);
            pending_transfers[i].len = len;
            pending_transfers[i].can_id = can_id;
            pending_transfers[i].mode = mode;
            pending_transfers[i].pid = pid;
            pending_transfers[i].pending = true;
            pending_transfer_count++;
            return true;
        }
    }
    return false;  // Queue full
}

void ecu_simClass::isotp_process_transfers(void) {
    // Timeout check for flow control
    if((isotp_tx.state == ISOTP_WAIT_FC || isotp_tx.state == ISOTP_WAIT_NEXT_FC) &&
       (millis() - isotp_tx.fc_wait_start) > 1000) {  // 1 second timeout
        isotp_tx.state = ISOTP_IDLE;  // Abort transfer
    }

    // Continue sending consecutive frames if in progress
    if(isotp_tx.state == ISOTP_SENDING_CF) {
        isotp_send_consecutive_frame();
    }

    // Check if current transfer is complete and we have pending transfers
    if(isotp_tx.state == ISOTP_IDLE && pending_transfer_count > 0) {
        // Start next pending transfer
        for(int i = 0; i < MAX_PENDING_TRANSFERS; i++) {
            if(pending_transfers[i].pending) {
                // Found a pending transfer - start it
                isotp_init_transfer(pending_transfers[i].data,
                                  pending_transfers[i].len,
                                  pending_transfers[i].can_id,
                                  pending_transfers[i].mode,
                                  pending_transfers[i].pid);
                isotp_send_first_frame();

                // Mark this transfer as started
                pending_transfers[i].pending = false;
                pending_transfer_count--;
                break;  // Only start one at a time
            }
        }
    }
}
