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
     
     // Handle ISO-TP Flow Control frames
     if (can_MsgRx.id >= 0x7E0 && can_MsgRx.id <= 0x7E7 &&
         (can_MsgRx.buf[0] & 0xF0) == ISO_TP_FLOW_CONTROL) {
         // Flow control received - pass to ISO-TP handler
         isotp_handle_flow_control(can_MsgRx.buf);
         return 0;  // Flow control processed
     }

     if (can_MsgRx.id == PID_REQUEST)
     {
       digitalWrite(LED_green, HIGH);
       flash_led_tick = 0;

       // Check if this is an ISO-TP First Frame from tester (for receiving multi-frame requests)
       if ((can_MsgRx.buf[0] & 0xF0) == ISO_TP_FIRST_FRAME) {
           // Multi-frame request from tester - send flow control
           CAN_message_t flowControl;
           flowControl.id = PID_REPLY_ENGINE;
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

        if(can_MsgRx.buf[1] == MODE3) // Request trouble codes
        {
            if(ecu.dtc == false){
                can_MsgTx.buf[0] = 0x02; 
                can_MsgTx.buf[1] = MODE3_RESPONSE;    
                can_MsgTx.buf[2] = 0x00;  
             }else{
                can_MsgTx.buf[0] = 0x06; 
                can_MsgTx.buf[1] = MODE3_RESPONSE;    
                can_MsgTx.buf[2] = 0x02;  
                can_MsgTx.buf[3] = 0x01;  
                can_MsgTx.buf[4] = 0x00;                
                can_MsgTx.buf[5] = 0x02;
                can_MsgTx.buf[6] = 0x00;                
             }
             can_MsgTx.id = PID_REPLY;  //7E8
             can_MsgTx.len = 8; 
             can1.write(can_MsgTx);
        }
      
        if(can_MsgRx.buf[1] == MODE4) // Clear trouble codes, clear Check engine light
        {
            ecu.dtc = false;
            digitalWrite(LED_red, LOW);
            // Clear freeze frame data
            freeze_frame[0].data_stored = false;
            freeze_frame[1].data_stored = false;
            can_MsgTx.buf[0] = 0x00;
            can_MsgTx.buf[1] = MODE4_RESPONSE;
            can_MsgTx.id = PID_REPLY;
            can_MsgTx.len = 8;
            can1.write(can_MsgTx);
        }

        if(can_MsgRx.buf[1] == MODE2) // Freeze frame data
        {
            can_MsgTx.id = PID_REPLY;
            can_MsgTx.len = 8;
            can_MsgTx.buf[1] = MODE2_RESPONSE;

            // Standard OBD-II: buf[2]=PID, buf[3]=Frame (optional, defaults to 0x00)
            // Check buf[0] for actual data length, or use buf[3] if provided
            uint8_t frame_num = (can_MsgRx.buf[0] >= 3) ? can_MsgRx.buf[3] : 0x00;

            // Support frame 0 and 1 (we have 2 DTCs with freeze frames)
            if(frame_num <= 1 && freeze_frame[frame_num].data_stored)
            {
                // Use stored freeze frame data, not current sensor values
                switch(can_MsgRx.buf[2])  // PID is in buf[2]
                {
                    case PID_SUPPORTED:
                        can_MsgTx.buf[0] = 0x06;
                        can_MsgTx.buf[2] = PID_SUPPORTED;
                        can_MsgTx.buf[3] = 0xE8;  // Same PIDs as Mode 1
                        can_MsgTx.buf[4] = 0x19;
                        can_MsgTx.buf[5] = 0x30;
                        can_MsgTx.buf[6] = 0x12;
                        can1.write(can_MsgTx);
                        break;

                    case ENGINE_RPM:
                        can_MsgTx.buf[0] = 0x04;
                        can_MsgTx.buf[2] = ENGINE_RPM;
                        can_MsgTx.buf[3] = (freeze_frame[frame_num].engine_rpm & 0xff00) >> 8;
                        can_MsgTx.buf[4] = freeze_frame[frame_num].engine_rpm & 0x00ff;
                        can1.write(can_MsgTx);
                        break;

                    case ENGINE_COOLANT_TEMP:
                        can_MsgTx.buf[0] = 0x03;
                        can_MsgTx.buf[2] = ENGINE_COOLANT_TEMP;
                        can_MsgTx.buf[3] = freeze_frame[frame_num].coolant_temp;
                        can1.write(can_MsgTx);
                        break;

                    case VEHICLE_SPEED:
                        can_MsgTx.buf[0] = 0x03;
                        can_MsgTx.buf[2] = VEHICLE_SPEED;
                        can_MsgTx.buf[3] = freeze_frame[frame_num].vehicle_speed;
                        can1.write(can_MsgTx);
                        break;

                    case MAF_SENSOR:
                        can_MsgTx.buf[0] = 0x04;
                        can_MsgTx.buf[2] = MAF_SENSOR;
                        can_MsgTx.buf[3] = (freeze_frame[frame_num].maf_airflow & 0xff00) >> 8;
                        can_MsgTx.buf[4] = freeze_frame[frame_num].maf_airflow & 0x00ff;
                        can1.write(can_MsgTx);
                        break;

                    case O2_VOLTAGE:
                        can_MsgTx.buf[0] = 0x04;
                        can_MsgTx.buf[2] = O2_VOLTAGE;
                        can_MsgTx.buf[3] = freeze_frame[frame_num].o2_voltage & 0x00ff;
                        can_MsgTx.buf[4] = (freeze_frame[frame_num].o2_voltage & 0xff00) >> 8;
                        can1.write(can_MsgTx);
                        break;

                    case THROTTLE:
                        can_MsgTx.buf[0] = 0x03;
                        can_MsgTx.buf[2] = THROTTLE;
                        can_MsgTx.buf[3] = freeze_frame[frame_num].throttle_position;
                        can1.write(can_MsgTx);
                        break;
                }
            }
            else
            {
                // No freeze frame data stored for this frame number
                can_MsgTx.buf[0] = 0x00;  // No data available
                can1.write(can_MsgTx);
            }
        }

        if(can_MsgRx.buf[1] == MODE1) // Live data
        {
            // Mode 1 responses - simulate multiple ECUs for scanner detection
            can_MsgTx.len = 8;
            can_MsgTx.buf[1] = MODE1_RESPONSE;

            // Determine which ECU responds based on PID
            // For PID 00 (supported PIDs), multiple ECUs respond
            // For other PIDs, only relevant ECU responds
            bool sendEngineResponse = false;
            bool sendTransResponse = false;

            // Check which PIDs this ECU should respond to
            if(can_MsgRx.buf[2] == PID_SUPPORTED || can_MsgRx.buf[2] == PID_20_SUPPORTED ||
               can_MsgRx.buf[2] == PID_40_SUPPORTED) {
                // All ECUs respond to supported PID requests
                sendEngineResponse = true;
                sendTransResponse = true;
            } else {
                // Engine ECU handles most PIDs
                sendEngineResponse = true;
            }

            // Realistic driving simulation with states
            enum DriveState { IDLE, CITY, ACCELERATING, HIGHWAY, BRAKING };
            static DriveState driveState = IDLE;
            static unsigned long stateChangeTime = 0;
            static unsigned long lastUpdate = 0;
            static uint16_t currentRPM = 0x0990;  // 612 RPM idle
            static uint8_t currentSpeed = 0x00;
            static uint8_t currentLoad = 0x3E;
            static uint8_t currentThrottle = 0x1E;
            static uint8_t o2_voltage = 0x80;  // Oscillating O2 sensor

            // Update driving state every few seconds
            if(millis() - stateChangeTime > 10000) {  // Change state every 10 seconds
                int nextState = random(0, 5);
                driveState = (DriveState)nextState;
                stateChangeTime = millis();
            }

            // Update values based on driving state (every 100ms for smooth changes)
            if(millis() - lastUpdate > 100) {
                switch(driveState) {
                    case IDLE:
                        // Idle: 600-650 RPM, 0 km/h
                        currentRPM = 0x0990 + random(-20, 30);  // 600-650 RPM
                        currentSpeed = 0x00;
                        currentLoad = 0x3D + random(-2, 3);     // ~24%
                        currentThrottle = 0x1E;                 // 11.8%
                        break;

                    case CITY:
                        // City: 1000-1500 RPM, 15-50 km/h
                        currentRPM = 0x0FA0 + random(-50, 100);  // ~1000-1500 RPM
                        currentSpeed = 0x0F + random(0, 0x23);   // 15-50 km/h
                        currentLoad = 0x50 + random(-5, 10);     // ~35%
                        currentThrottle = 0x40;                  // 25%
                        break;

                    case ACCELERATING:
                        // Accelerating: 1800-2500 RPM, increasing speed
                        currentRPM = 0x1C20 + random(-100, 200); // 1800-2500 RPM
                        if(currentSpeed < 0x50) currentSpeed += 2;  // Increase speed
                        currentLoad = 0x80 + random(-10, 10);    // ~50%
                        currentThrottle = 0x80;                  // 50%
                        break;

                    case HIGHWAY:
                        // Highway: 1600-1700 RPM, 78-79 km/h (from Mercedes data)
                        currentRPM = 0x1900 + random(-50, 50);   // ~1600 RPM
                        currentSpeed = 0x4E + random(-1, 2);     // 78-79 km/h
                        currentLoad = 0x60 + random(-5, 5);      // ~38%
                        currentThrottle = 0x4A;                  // 29%
                        break;

                    case BRAKING:
                        // Braking: decreasing RPM and speed
                        if(currentRPM > 0x0990) currentRPM -= 0x50;  // Decrease RPM
                        if(currentSpeed > 0) currentSpeed -= 3;       // Decrease speed
                        currentLoad = 0x20;                           // Low load
                        currentThrottle = 0x00;                       // 0% throttle
                        break;
                }

                // O2 sensor oscillation (rich/lean cycling around stoichiometric)
                o2_voltage = 0x70 + random(0, 0x20);  // 0.35-0.45V typical oscillation

                lastUpdate = millis();
            }

            switch(can_MsgRx.buf[2])  // PID is in buf[2]
            {
                case PID_SUPPORTED:  // 0x00 - PIDs 01-20
                    if(sendEngineResponse) {
                        can_MsgTx.id = PID_REPLY_ENGINE;
                        can_MsgTx.buf[0] = 0x06;
                        can_MsgTx.buf[2] = PID_SUPPORTED;
                        can_MsgTx.buf[3] = 0xBF;  // Engine supports many PIDs
                        can_MsgTx.buf[4] = 0xBE;
                        can_MsgTx.buf[5] = 0xA8;
                        can_MsgTx.buf[6] = 0x93;
                        can1.write(can_MsgTx);
                    }

                    if(sendTransResponse) {
                        delay(5);  // Small delay between ECU responses
                        can_MsgTx.id = PID_REPLY_TRANS;
                        can_MsgTx.buf[0] = 0x06;
                        can_MsgTx.buf[2] = PID_SUPPORTED;
                        can_MsgTx.buf[3] = 0x18;  // Trans supports fewer PIDs
                        can_MsgTx.buf[4] = 0x00;
                        can_MsgTx.buf[5] = 0x00;
                        can_MsgTx.buf[6] = 0x00;
                        can1.write(can_MsgTx);
                    }
                    break;

                case PID_20_SUPPORTED:  // 0x20 - PIDs 21-40
                    if(sendEngineResponse) {
                        can_MsgTx.id = PID_REPLY_ENGINE;
                        can_MsgTx.buf[0] = 0x06;
                        can_MsgTx.buf[2] = PID_20_SUPPORTED;
                        can_MsgTx.buf[3] = 0xA0;
                        can_MsgTx.buf[4] = 0x07;
                        can_MsgTx.buf[5] = 0xF1;
                        can_MsgTx.buf[6] = 0x19;
                        can1.write(can_MsgTx);
                    }

                    if(sendTransResponse) {
                        delay(5);
                        can_MsgTx.id = PID_REPLY_TRANS;
                        can_MsgTx.buf[0] = 0x06;
                        can_MsgTx.buf[2] = PID_20_SUPPORTED;
                        can_MsgTx.buf[3] = 0x00;  // Trans doesn't support these
                        can_MsgTx.buf[4] = 0x00;
                        can_MsgTx.buf[5] = 0x00;
                        can_MsgTx.buf[6] = 0x00;
                        can1.write(can_MsgTx);
                    }
                    break;

                case PID_40_SUPPORTED:  // 0x40 - PIDs 41-60
                    if(sendEngineResponse) {
                        can_MsgTx.id = PID_REPLY_ENGINE;
                        can_MsgTx.buf[0] = 0x06;
                        can_MsgTx.buf[2] = PID_40_SUPPORTED;
                        can_MsgTx.buf[3] = 0xFE;
                        can_MsgTx.buf[4] = 0xD0;
                        can_MsgTx.buf[5] = 0x85;
                        can_MsgTx.buf[6] = 0x00;
                        can1.write(can_MsgTx);
                    }

                    if(sendTransResponse) {
                        delay(5);
                        can_MsgTx.id = PID_REPLY_TRANS;
                        can_MsgTx.buf[0] = 0x06;
                        can_MsgTx.buf[2] = PID_40_SUPPORTED;
                        can_MsgTx.buf[3] = 0x00;  // Trans doesn't support these
                        can_MsgTx.buf[4] = 0x00;
                        can_MsgTx.buf[5] = 0x00;
                        can_MsgTx.buf[6] = 0x00;
                        can1.write(can_MsgTx);
                    }
                    break;

                case MONITOR_STATUS:  // 0x01
                    can_MsgTx.id = PID_REPLY_ENGINE;
                    can_MsgTx.buf[0] = 0x06;
                    can_MsgTx.buf[2] = MONITOR_STATUS;
                    can_MsgTx.buf[3] = 0x00;  // From Mercedes: 0007E500
                    can_MsgTx.buf[4] = 0x07;
                    can_MsgTx.buf[5] = 0xE5;
                    can_MsgTx.buf[6] = 0x00;
                    can1.write(can_MsgTx);
                    break;

                case FUEL_SYSTEM_STATUS:  // 0x03
                    can_MsgTx.id = PID_REPLY_ENGINE;
                    can_MsgTx.buf[0] = 0x04;
                    can_MsgTx.buf[2] = FUEL_SYSTEM_STATUS;
                    can_MsgTx.buf[3] = 0x02;  // From Mercedes: 0200
                    can_MsgTx.buf[4] = 0x00;
                    can1.write(can_MsgTx);
                    break;

                case CALCULATED_LOAD:  // 0x04
                    can_MsgTx.id = PID_REPLY_ENGINE;
                    can_MsgTx.buf[0] = 0x03;
                    can_MsgTx.buf[2] = CALCULATED_LOAD;
                    can_MsgTx.buf[3] = currentLoad;  // Dynamic load value
                    can1.write(can_MsgTx);
                    break;

                case ENGINE_COOLANT_TEMP:  // 0x05
                    can_MsgTx.id = PID_REPLY_ENGINE;
                    can_MsgTx.buf[0] = 0x03;
                    can_MsgTx.buf[2] = ENGINE_COOLANT_TEMP;
                    can_MsgTx.buf[3] = 0x87;  // From Mercedes: 95°C (0x87)
                    can1.write(can_MsgTx);
                    break;

                case SHORT_FUEL_TRIM_1:  // 0x06
                    can_MsgTx.id = PID_REPLY_ENGINE;
                    can_MsgTx.buf[0] = 0x03;
                    can_MsgTx.buf[2] = SHORT_FUEL_TRIM_1;
                    can_MsgTx.buf[3] = 0x7F;  // From Mercedes: -0.8%
                    can1.write(can_MsgTx);
                    break;

                case LONG_FUEL_TRIM_1:  // 0x07
                    can_MsgTx.id = PID_REPLY_ENGINE;
                    can_MsgTx.buf[0] = 0x03;
                    can_MsgTx.buf[2] = LONG_FUEL_TRIM_1;
                    can_MsgTx.buf[3] = 0x83;  // From Mercedes: 2.3%
                    can1.write(can_MsgTx);
                    break;

                case SHORT_FUEL_TRIM_2:  // 0x08
                    can_MsgTx.buf[0] = 0x03;
                    can_MsgTx.buf[2] = SHORT_FUEL_TRIM_2;
                    can_MsgTx.buf[3] = 0x7F;  // From Mercedes: -0.8%
                    can1.write(can_MsgTx);
                    break;

                case LONG_FUEL_TRIM_2:  // 0x09
                    can_MsgTx.buf[0] = 0x03;
                    can_MsgTx.buf[2] = LONG_FUEL_TRIM_2;
                    can_MsgTx.buf[3] = 0x7B;  // From Mercedes: -3.9%
                    can1.write(can_MsgTx);
                    break;

                case INTAKE_PRESSURE:  // 0x0B
                    can_MsgTx.buf[0] = 0x03;
                    can_MsgTx.buf[2] = INTAKE_PRESSURE;
                    can_MsgTx.buf[3] = 0x21;  // From Mercedes: 33 kPa
                    can1.write(can_MsgTx);
                    break;

                case ENGINE_RPM:  // 0x0C
                    can_MsgTx.id = PID_REPLY_ENGINE;
                    can_MsgTx.buf[0] = 0x04;
                    can_MsgTx.buf[2] = ENGINE_RPM;
                    can_MsgTx.buf[3] = (currentRPM >> 8) & 0xFF;
                    can_MsgTx.buf[4] = currentRPM & 0xFF;
                    can1.write(can_MsgTx);
                    break;

                case VEHICLE_SPEED:  // 0x0D
                    can_MsgTx.id = PID_REPLY_ENGINE;
                    can_MsgTx.buf[0] = 0x03;
                    can_MsgTx.buf[2] = VEHICLE_SPEED;
                    can_MsgTx.buf[3] = currentSpeed;  // Dynamic speed value
                    can1.write(can_MsgTx);
                    break;

                case TIMING_ADVANCE:  // 0x0E
                    can_MsgTx.buf[0] = 0x03;
                    can_MsgTx.buf[2] = TIMING_ADVANCE;
                    can_MsgTx.buf[3] = 0x8C;  // From Mercedes: 6.0°
                    can1.write(can_MsgTx);
                    break;

                case INTAKE_AIR_TEMP:  // 0x0F
                    can_MsgTx.buf[0] = 0x03;
                    can_MsgTx.buf[2] = INTAKE_AIR_TEMP;
                    can_MsgTx.buf[3] = 0x65;  // From Mercedes: 61°C
                    can1.write(can_MsgTx);
                    break;

                case MAF_SENSOR:  // 0x10
                    {
                        // MAF scales with RPM - typical 2-25 g/s
                        uint16_t maf_value = (currentRPM >> 4) + random(-5, 6);  // Scale with RPM
                        can_MsgTx.id = PID_REPLY_ENGINE;
                        can_MsgTx.buf[0] = 0x04;
                        can_MsgTx.buf[2] = MAF_SENSOR;
                        can_MsgTx.buf[3] = (maf_value >> 8) & 0xFF;
                        can_MsgTx.buf[4] = maf_value & 0xFF;
                        can1.write(can_MsgTx);
                    }
                    break;

                case THROTTLE:  // 0x11
                    can_MsgTx.id = PID_REPLY_ENGINE;
                    can_MsgTx.buf[0] = 0x03;
                    can_MsgTx.buf[2] = THROTTLE;
                    can_MsgTx.buf[3] = currentThrottle;  // Dynamic throttle value
                    can1.write(can_MsgTx);
                    break;

                case O2_SENSORS_PRESENT:  // 0x13
                    can_MsgTx.buf[0] = 0x03;
                    can_MsgTx.buf[2] = O2_SENSORS_PRESENT;
                    can_MsgTx.buf[3] = 0x33;  // From Mercedes
                    can1.write(can_MsgTx);
                    break;

                case O2_SENSOR_2_B1:  // 0x15
                    can_MsgTx.buf[0] = 0x04;
                    can_MsgTx.buf[2] = O2_SENSOR_2_B1;
                    can_MsgTx.buf[3] = o2_voltage;  // Dynamic O2 voltage
                    can_MsgTx.buf[4] = 0xFF;        // Not used for trim
                    can1.write(can_MsgTx);
                    break;

                case O2_SENSOR_2_B2:  // 0x19
                    can_MsgTx.buf[0] = 0x04;
                    can_MsgTx.buf[2] = O2_SENSOR_2_B2;
                    can_MsgTx.buf[3] = o2_voltage + 5;  // Slightly different for Bank 2
                    can_MsgTx.buf[4] = 0xFF;            // Not used for trim
                    can1.write(can_MsgTx);
                    break;

                case OBD_STANDARD:  // 0x1C
                    can_MsgTx.buf[0] = 0x03;
                    can_MsgTx.buf[2] = OBD_STANDARD;
                    can_MsgTx.buf[3] = 0x03;  // From Mercedes
                    can1.write(can_MsgTx);
                    break;

                case ENGINE_RUN_TIME:  // 0x1F
                    can_MsgTx.buf[0] = 0x04;
                    can_MsgTx.buf[2] = ENGINE_RUN_TIME;
                    can_MsgTx.buf[3] = 0x2A;  // From Mercedes: 10926 sec
                    can_MsgTx.buf[4] = 0xAE;
                    can1.write(can_MsgTx);
                    break;

                case DISTANCE_WITH_MIL:  // 0x21
                    can_MsgTx.buf[0] = 0x04;
                    can_MsgTx.buf[2] = DISTANCE_WITH_MIL;
                    can_MsgTx.buf[3] = 0x00;  // From Mercedes: 0 km
                    can_MsgTx.buf[4] = 0x00;
                    can1.write(can_MsgTx);
                    break;

                case FUEL_RAIL_PRESSURE:  // 0x23
                    can_MsgTx.buf[0] = 0x04;
                    can_MsgTx.buf[2] = FUEL_RAIL_PRESSURE;
                    can_MsgTx.buf[3] = 0x05;  // From Mercedes: 117.4 kPa
                    can_MsgTx.buf[4] = 0xCE;
                    can1.write(can_MsgTx);
                    break;

                case EVAP_PURGE:  // 0x2E
                    can_MsgTx.buf[0] = 0x03;
                    can_MsgTx.buf[2] = EVAP_PURGE;
                    can_MsgTx.buf[3] = 0x79;  // From Mercedes: 47.5%
                    can1.write(can_MsgTx);
                    break;

                case FUEL_LEVEL:  // 0x2F
                    can_MsgTx.buf[0] = 0x03;
                    can_MsgTx.buf[2] = FUEL_LEVEL;
                    can_MsgTx.buf[3] = 0x39;  // From Mercedes: 22.4%
                    can1.write(can_MsgTx);
                    break;

                case WARM_UPS:  // 0x30
                    can_MsgTx.buf[0] = 0x03;
                    can_MsgTx.buf[2] = WARM_UPS;
                    can_MsgTx.buf[3] = 0xFF;  // From Mercedes
                    can1.write(can_MsgTx);
                    break;

                case DISTANCE_SINCE_CLR:  // 0x31
                    can_MsgTx.buf[0] = 0x04;
                    can_MsgTx.buf[2] = DISTANCE_SINCE_CLR;
                    can_MsgTx.buf[3] = 0xFF;  // From Mercedes: 65535 km
                    can_MsgTx.buf[4] = 0xFF;
                    can1.write(can_MsgTx);
                    break;

                case EVAP_VAPOR_PRESS:  // 0x32
                    can_MsgTx.buf[0] = 0x04;
                    can_MsgTx.buf[2] = EVAP_VAPOR_PRESS;
                    can_MsgTx.buf[3] = 0xFD;  // From Mercedes
                    can_MsgTx.buf[4] = 0xDD;
                    can1.write(can_MsgTx);
                    break;

                case BAROMETRIC_PRESS:  // 0x33
                    can_MsgTx.buf[0] = 0x03;
                    can_MsgTx.buf[2] = BAROMETRIC_PRESS;
                    can_MsgTx.buf[3] = 0x62;  // From Mercedes: 98 kPa
                    can1.write(can_MsgTx);
                    break;

                case O2_SENSOR_1_B1:  // 0x34
                    can_MsgTx.buf[0] = 0x06;
                    can_MsgTx.buf[2] = O2_SENSOR_1_B1;
                    can_MsgTx.buf[3] = 0x80;  // From Mercedes
                    can_MsgTx.buf[4] = 0xA7;
                    can_MsgTx.buf[5] = 0x80;
                    can_MsgTx.buf[6] = 0x00;
                    can1.write(can_MsgTx);
                    break;

                case O2_SENSOR_5_B2:  // 0x38
                    can_MsgTx.buf[0] = 0x06;
                    can_MsgTx.buf[2] = O2_SENSOR_5_B2;
                    can_MsgTx.buf[3] = 0x80;  // From Mercedes
                    can_MsgTx.buf[4] = 0x37;
                    can_MsgTx.buf[5] = 0x7F;
                    can_MsgTx.buf[6] = 0xFD;
                    can1.write(can_MsgTx);
                    break;

                case CAT_TEMP_B1S1:  // 0x3C
                    can_MsgTx.buf[0] = 0x04;
                    can_MsgTx.buf[2] = CAT_TEMP_B1S1;
                    can_MsgTx.buf[3] = 0x11;  // From Mercedes
                    can_MsgTx.buf[4] = 0x7F;
                    can1.write(can_MsgTx);
                    break;

                case CAT_TEMP_B2S1:  // 0x3D
                    can_MsgTx.buf[0] = 0x04;
                    can_MsgTx.buf[2] = CAT_TEMP_B2S1;
                    can_MsgTx.buf[3] = 0x11;  // From Mercedes
                    can_MsgTx.buf[4] = 0x7E;
                    can1.write(can_MsgTx);
                    break;

                case MONITOR_STATUS_CYC:  // 0x41
                    can_MsgTx.buf[0] = 0x06;
                    can_MsgTx.buf[2] = MONITOR_STATUS_CYC;
                    can_MsgTx.buf[3] = 0x00;  // From Mercedes
                    can_MsgTx.buf[4] = 0x05;
                    can_MsgTx.buf[5] = 0xE0;
                    can_MsgTx.buf[6] = 0x24;
                    can1.write(can_MsgTx);
                    break;

                case CONTROL_MOD_VOLT:  // 0x42
                    can_MsgTx.buf[0] = 0x04;
                    can_MsgTx.buf[2] = CONTROL_MOD_VOLT;
                    can_MsgTx.buf[3] = 0x33;  // From Mercedes: 13.31V
                    can_MsgTx.buf[4] = 0xFF;
                    can1.write(can_MsgTx);
                    break;

                case ABSOLUTE_LOAD:  // 0x43
                    can_MsgTx.buf[0] = 0x04;
                    can_MsgTx.buf[2] = ABSOLUTE_LOAD;
                    can_MsgTx.buf[3] = 0x00;  // From Mercedes: 17.6%
                    can_MsgTx.buf[4] = 0x2D;
                    can1.write(can_MsgTx);
                    break;

                case COMMANDED_EQUIV:  // 0x44
                    can_MsgTx.buf[0] = 0x04;
                    can_MsgTx.buf[2] = COMMANDED_EQUIV;
                    can_MsgTx.buf[3] = 0x7F;  // From Mercedes
                    can_MsgTx.buf[4] = 0xFF;
                    can1.write(can_MsgTx);
                    break;

                case REL_THROTTLE_POS:  // 0x45
                    can_MsgTx.buf[0] = 0x03;
                    can_MsgTx.buf[2] = REL_THROTTLE_POS;
                    can_MsgTx.buf[3] = currentThrottle >> 2;  // Relative throttle (1/4 of absolute)
                    can1.write(can_MsgTx);
                    break;

                case AMBIENT_AIR_TEMP:  // 0x46
                    can_MsgTx.buf[0] = 0x03;
                    can_MsgTx.buf[2] = AMBIENT_AIR_TEMP;
                    can_MsgTx.buf[3] = 0x4E;  // From Mercedes: 38°C
                    can1.write(can_MsgTx);
                    break;

                case THROTTLE_POS_B:  // 0x47
                    can_MsgTx.buf[0] = 0x03;
                    can_MsgTx.buf[2] = THROTTLE_POS_B;
                    can_MsgTx.buf[3] = currentThrottle;  // Same as throttle A
                    can1.write(can_MsgTx);
                    break;

                case ACCEL_POS_D:  // 0x49
                    can_MsgTx.buf[0] = 0x03;
                    can_MsgTx.buf[2] = ACCEL_POS_D;
                    can_MsgTx.buf[3] = 0x11;  // From Mercedes
                    can1.write(can_MsgTx);
                    break;

                case ACCEL_POS_E:  // 0x4A
                    can_MsgTx.buf[0] = 0x03;
                    can_MsgTx.buf[2] = ACCEL_POS_E;
                    can_MsgTx.buf[3] = 0x11;  // From Mercedes
                    can1.write(can_MsgTx);
                    break;

                case COMMANDED_THROTTLE:  // 0x4C
                    can_MsgTx.buf[0] = 0x03;
                    can_MsgTx.buf[2] = COMMANDED_THROTTLE;
                    can_MsgTx.buf[3] = currentThrottle >> 1;  // Half of actual throttle
                    can1.write(can_MsgTx);
                    break;

                case FUEL_TYPE:  // 0x51
                    can_MsgTx.buf[0] = 0x03;
                    can_MsgTx.buf[2] = FUEL_TYPE;
                    can_MsgTx.buf[3] = 0x01;  // From Mercedes
                    can1.write(can_MsgTx);
                    break;

                case SHORT_O2_TRIM_B1:  // 0x56
                    can_MsgTx.buf[0] = 0x03;
                    can_MsgTx.buf[2] = SHORT_O2_TRIM_B1;
                    can_MsgTx.buf[3] = 0x7E;  // From Mercedes
                    can1.write(can_MsgTx);
                    break;

                case SHORT_O2_TRIM_B2:  // 0x58
                    can_MsgTx.buf[0] = 0x03;
                    can_MsgTx.buf[2] = SHORT_O2_TRIM_B2;
                    can_MsgTx.buf[3] = 0x7F;  // From Mercedes
                    can1.write(can_MsgTx);
                    break;

                default:
                    // For all other PIDs, use engine ECU ID
                    can_MsgTx.id = PID_REPLY_ENGINE;
                    break;
            }
        }

        if(can_MsgRx.buf[1] == MODE9) // Vehicle information
        {
            // Default to Engine ECU response
            can_MsgTx.id = PID_REPLY_ENGINE;
            can_MsgTx.len = 8;
            can_MsgTx.buf[1] = MODE9_RESPONSE;

            // Check for ISO-TP flow control frames
            if((can_MsgRx.buf[0] & 0xF0) == ISO_TP_FLOW_CONTROL) {
                // Handle flow control - continue sending consecutive frames
                // This would be implemented based on pending multi-frame transfers
                // For now, we'll handle it in the multi-frame sections
                return 0;
            }

            switch(can_MsgRx.buf[2])  // PID is in buf[2]
            {
                case VEH_INFO_SUPPORTED:  // 0x00 - Supported PIDs
                    // Simulate multiple ECUs responding with different CAN IDs
                    // Engine ECU response (0x7E8)
                    can_MsgTx.id = PID_REPLY_ENGINE;
                    can_MsgTx.buf[0] = 0x06;  // Single frame, 6 bytes
                    can_MsgTx.buf[2] = VEH_INFO_SUPPORTED;
                    can_MsgTx.buf[3] = 0x55;  // Supports: 0x02, 0x04, 0x06, 0x08, 0x0A, 0x14
                    can_MsgTx.buf[4] = 0x40;  // Bit 6 set
                    can_MsgTx.buf[5] = 0x10;  // Bit 4 set
                    can_MsgTx.buf[6] = 0x00;
                    can_MsgTx.buf[7] = 0x00;  // Padding
                    can1.write(can_MsgTx);

                    // Small delay between ECU responses (realistic timing)
                    delay(5);

                    // Transmission ECU response (0x7E9) - different supported PIDs
                    can_MsgTx.id = PID_REPLY_TRANS;
                    can_MsgTx.buf[0] = 0x06;  // Single frame, 6 bytes
                    can_MsgTx.buf[2] = VEH_INFO_SUPPORTED;
                    can_MsgTx.buf[3] = 0x14;  // Supports: 0x02, 0x04 (different from engine)
                    can_MsgTx.buf[4] = 0x40;
                    can_MsgTx.buf[5] = 0x00;
                    can_MsgTx.buf[6] = 0x00;
                    can_MsgTx.buf[7] = 0x00;  // Padding
                    can1.write(can_MsgTx);
                    break;

                case VIN_REQUEST:  // 0x02 - Vehicle Identification Number
                    {
                        // Only start transfer if not already in progress
                        if(isotp_tx.state != ISOTP_IDLE) break;

                        // VIN: 4JGDA5HB7JB158144 (17 chars)
                        uint8_t vin_data[20];  // 3 header + 17 VIN
                        vin_data[0] = MODE9_RESPONSE;
                        vin_data[1] = VIN_REQUEST;
                        vin_data[2] = 0x01;  // 1 data item
                        // VIN characters
                        const char* vin = "4JGDA5HB7JB158144";
                        for(int i = 0; i < 17; i++) {
                            vin_data[i+3] = vin[i];
                        }

                        // Initialize ISO-TP transfer
                        isotp_init_transfer(vin_data, 20, PID_REPLY_ENGINE, MODE9, VIN_REQUEST);
                        isotp_send_first_frame();
                    }
                    break;

                case CAL_ID_REQUEST:  // 0x04 - Calibration ID
                    {
                        // Only start transfer if not already in progress
                        if(isotp_tx.state != ISOTP_IDLE) break;

                        // Cal ID: 2769011200190170 (16 chars)
                        uint8_t cal_data[19];  // 3 header + 16 cal ID
                        cal_data[0] = MODE9_RESPONSE;
                        cal_data[1] = CAL_ID_REQUEST;
                        cal_data[2] = 0x01;  // 1 data item
                        // Calibration ID
                        const char* cal_id = "2769011200190170";
                        for(int i = 0; i < 16; i++) {
                            cal_data[i+3] = cal_id[i];
                        }

                        // Initialize ISO-TP transfer
                        isotp_init_transfer(cal_data, 19, PID_REPLY_ENGINE, MODE9, CAL_ID_REQUEST);
                        isotp_send_first_frame();
                    }
                    break;

                case CVN_REQUEST:  // 0x06 - Calibration Verification Number
                    // Single frame response
                    can_MsgTx.id = PID_REPLY_ENGINE;
                    can_MsgTx.buf[0] = 0x06;  // Single frame, 6 bytes
                    can_MsgTx.buf[2] = CVN_REQUEST;
                    can_MsgTx.buf[3] = 0x01;  // 1 CVN
                    can_MsgTx.buf[4] = 0xEB;
                    can_MsgTx.buf[5] = 0x85;
                    can_MsgTx.buf[6] = 0x49;
                    can_MsgTx.buf[7] = 0x39;
                    can1.write(can_MsgTx);
                    break;

                case ECU_NAME_REQUEST:  // 0x0A - ECU Name
                    {
                        // Only start transfer if not already in progress
                        if(isotp_tx.state != ISOTP_IDLE) break;

                        // ECU Name: "ECM-EngineControl"
                        uint8_t name_data[23];  // Total message
                        name_data[0] = MODE9_RESPONSE;
                        name_data[1] = ECU_NAME_REQUEST;
                        name_data[2] = 0x01;  // 1 data item
                        // ECU Name with separator
                        name_data[3] = 'E';
                        name_data[4] = 'C';
                        name_data[5] = 'M';
                        name_data[6] = 0x00;  // Separator
                        name_data[7] = '-';
                        const char* name_rest = "EngineControl";
                        for(int i = 0; i < 13; i++) {
                            name_data[i+8] = name_rest[i];
                        }
                        name_data[21] = 0x00;  // Null terminator
                        name_data[22] = 0x00;  // Padding

                        // Initialize ISO-TP transfer
                        isotp_init_transfer(name_data, 23, PID_REPLY_ENGINE, MODE9, ECU_NAME_REQUEST);
                        isotp_send_first_frame();
                    }
                    break;

                case PERF_TRACK_REQUEST:  // 0x08 - Performance Tracking
                    {
                        // Only start transfer if not already in progress
                        if(isotp_tx.state != ISOTP_IDLE) break;

                        // Performance tracking data: 43 bytes total
                        uint8_t perf_data[43];
                        perf_data[0] = MODE9_RESPONSE;
                        perf_data[1] = PERF_TRACK_REQUEST;

                        // Real performance tracking data from Mercedes-Benz
                        uint8_t perf_values[] = {
                            0x14, 0x10, 0x62, 0x2E, 0x4C, 0x17, 0x69, 0x10,
                            0x62, 0x17, 0x04, 0xD0, 0x10, 0x10, 0x06, 0x2E,
                            0x00, 0x17, 0x00, 0x00, 0xD0, 0x00, 0x10, 0x00,
                            0x2E, 0x00, 0x11, 0x00, 0x00, 0xD0, 0x00, 0x0E,
                            0x00, 0x00, 0x00, 0x00, 0x00, 0xD0, 0x00, 0x00,
                            0x00
                        };

                        for(int i = 0; i < 41; i++) {
                            perf_data[i+2] = perf_values[i];
                        }

                        // Initialize ISO-TP transfer
                        isotp_init_transfer(perf_data, 43, PID_REPLY_ENGINE, MODE9, PERF_TRACK_REQUEST);
                        isotp_send_first_frame();
                    }
                    break;

                case AUX_IO_REQUEST:  // 0x14 - Auxiliary I/O Status
                    // Single frame response
                    can_MsgTx.id = PID_REPLY_ENGINE;
                    can_MsgTx.buf[0] = 0x05;  // Single frame, 5 bytes
                    can_MsgTx.buf[2] = AUX_IO_REQUEST;
                    can_MsgTx.buf[3] = 0x01;  // 1 data item
                    can_MsgTx.buf[4] = 0x00;
                    can_MsgTx.buf[5] = 0x18;
                    can_MsgTx.buf[6] = 0x00;  // Padding
                    can_MsgTx.buf[7] = 0x00;  // Padding
                    can1.write(can_MsgTx);
                    break;
            }
        }

        if(can_MsgRx.buf[1] == MODE1)
        {
            // This appears to be duplicate Mode 1 handling - remove it
            // The main Mode 1 handler above should handle everything
            // This section seems to be legacy code
            can_MsgTx.id = PID_REPLY_ENGINE;  // Use proper ECU ID
            can_MsgTx.len = 8;
            can_MsgTx.buf[1] = MODE1_RESPONSE;

            switch(can_MsgRx.buf[2])
            {   /* Details from http://en.wikipedia.org/wiki/OBD-II_PIDs */
                case PID_SUPPORTED:
                    can_MsgTx.buf[0] = 0x06;
                    can_MsgTx.buf[2] = PID_SUPPORTED;
                    can_MsgTx.buf[3] = 0xBF;  // Real vehicle: BFBEA893
                    can_MsgTx.buf[4] = 0xBE;
                    can_MsgTx.buf[5] = 0xA8;
                    can_MsgTx.buf[6] = 0x93;
                    can1.write(can_MsgTx);
                    break;
                
                case MONITOR_STATUS:
                    can_MsgTx.buf[0] = 0x05;
                    can_MsgTx.buf[2] = MONITOR_STATUS;
                    if(ecu.dtc == 1) can_MsgTx.buf[3] = 0x82;
                        else can_MsgTx.buf[3] = 0x00;

                    can_MsgTx.buf[4] = 0x07;  // Real vehicle: 0007E500
                    can_MsgTx.buf[5] = 0xE5;
                    can1.write(can_MsgTx);
                    break;

                case FUEL_SYSTEM_STATUS:      // 0x03 - Fuel system status
                    can_MsgTx.buf[0] = 0x03;
                    can_MsgTx.buf[2] = FUEL_SYSTEM_STATUS;
                    can_MsgTx.buf[3] = 0x02;  // Closed loop
                    can1.write(can_MsgTx);
                    break;

                case CALCULATED_LOAD:         // 0x04 - Calculated engine load
                    can_MsgTx.buf[0] = 0x03;
                    can_MsgTx.buf[2] = CALCULATED_LOAD;
                    can_MsgTx.buf[3] = map(ecu.throttle_position, 0, 255, 0, 100); // Based on throttle
                    can1.write(can_MsgTx);
                    break;

                case SHORT_FUEL_TRIM_1:       // 0x06
                    can_MsgTx.buf[0] = 0x03;
                    can_MsgTx.buf[2] = SHORT_FUEL_TRIM_1;
                    can_MsgTx.buf[3] = 0x80;  // 0% trim (128 = 0%)
                    can1.write(can_MsgTx);
                    break;

                case LONG_FUEL_TRIM_1:        // 0x07
                    can_MsgTx.buf[0] = 0x03;
                    can_MsgTx.buf[2] = LONG_FUEL_TRIM_1;
                    can_MsgTx.buf[3] = 0x83;  // +2.3% trim
                    can1.write(can_MsgTx);
                    break;

                case SHORT_FUEL_TRIM_2:       // 0x08
                    can_MsgTx.buf[0] = 0x03;
                    can_MsgTx.buf[2] = SHORT_FUEL_TRIM_2;
                    can_MsgTx.buf[3] = 0x7F;  // -0.8% trim
                    can1.write(can_MsgTx);
                    break;

                case LONG_FUEL_TRIM_2:        // 0x09
                    can_MsgTx.buf[0] = 0x03;
                    can_MsgTx.buf[2] = LONG_FUEL_TRIM_2;
                    can_MsgTx.buf[3] = 0x7A;  // -4.7% trim
                    can1.write(can_MsgTx);
                    break;

                case INTAKE_PRESSURE:         // 0x0B - MAP sensor
                    can_MsgTx.buf[0] = 0x03;
                    can_MsgTx.buf[2] = INTAKE_PRESSURE;
                    can_MsgTx.buf[3] = 33;    // 33 kPa at idle
                    can1.write(can_MsgTx);
                    break;

                case ENGINE_RPM:              //   ((A*256)+B)/4    [RPM]
                    can_MsgTx.buf[0] = 0x04;  
                    can_MsgTx.buf[2] = ENGINE_RPM; 
                    can_MsgTx.buf[3] = (ecu.engine_rpm & 0xff00) >> 8;
                    can_MsgTx.buf[4] = ecu.engine_rpm & 0x00ff;
                    can1.write(can_MsgTx);
                    break;
                               
                case ENGINE_COOLANT_TEMP:     //     A-40              [degree C]
                    can_MsgTx.buf[0] = 0x03;  
                    can_MsgTx.buf[2] = ENGINE_COOLANT_TEMP; 
                    can_MsgTx.buf[3] = ecu.coolant_temp;
                    can1.write(can_MsgTx);
                    break;
                               
                case VEHICLE_SPEED:         // A                  [km]
                    can_MsgTx.buf[0] = 0x03;
                    can_MsgTx.buf[2] = VEHICLE_SPEED;
                    can_MsgTx.buf[3] = ecu.vehicle_speed;
                    can1.write(can_MsgTx);
                    break;

                case TIMING_ADVANCE:        // 0x0E - Timing advance
                    can_MsgTx.buf[0] = 0x03;
                    can_MsgTx.buf[2] = TIMING_ADVANCE;
                    can_MsgTx.buf[3] = 0x8C;  // 14 degrees (140/2-64)
                    can1.write(can_MsgTx);
                    break;

                case INTAKE_AIR_TEMP:       // 0x0F - Intake air temperature
                    can_MsgTx.buf[0] = 0x03;
                    can_MsgTx.buf[2] = INTAKE_AIR_TEMP;
                    can_MsgTx.buf[3] = 0x65;  // 61°C (101-40)
                    can1.write(can_MsgTx);
                    break;

                case MAF_SENSOR:               // ((256*A)+B) / 100  [g/s]
                    can_MsgTx.buf[0] = 0x04;  
                    can_MsgTx.buf[2] = MAF_SENSOR; 
                    can_MsgTx.buf[3] = (ecu.maf_airflow & 0xff00) >> 8;
                    can_MsgTx.buf[4] =  ecu.maf_airflow & 0x00ff;
                    can1.write(can_MsgTx);
                    break;
    
                case O2_VOLTAGE:            // A * 0.005   (B-128) * 100/128 (if B==0xFF, sensor is not used in trim calc)
                    can_MsgTx.buf[0] = 0x04;  
                    can_MsgTx.buf[2] = O2_VOLTAGE; 
                    can_MsgTx.buf[3] = ecu.o2_voltage & 0x00ff;
                    can_MsgTx.buf[4] = (ecu.o2_voltage & 0xff00) >> 8;
                    can1.write(can_MsgTx);
                    break;;
                   
                case THROTTLE:            //
                    can_MsgTx.buf[0] = 0x03;
                    can_MsgTx.buf[2] = THROTTLE;
                    can_MsgTx.buf[3] = ecu.throttle_position;
                    can1.write(can_MsgTx);
                    Serial.print("Throttle: ");
                    Serial.println(ecu.throttle_position,HEX);
                    break;

                case O2_SENSORS_PRESENT:    // 0x13 - O2 sensors present
                    can_MsgTx.buf[0] = 0x03;
                    can_MsgTx.buf[2] = O2_SENSORS_PRESENT;
                    can_MsgTx.buf[3] = 0x33;  // Banks 1 & 2, sensors 1 & 2
                    can1.write(can_MsgTx);
                    break;

                case O2_SENSOR_2_B1:        // 0x15 - O2 Sensor 2 Bank 1
                    can_MsgTx.buf[0] = 0x04;
                    can_MsgTx.buf[2] = O2_SENSOR_2_B1;
                    can_MsgTx.buf[3] = 0x3C;  // 0.3V
                    can_MsgTx.buf[4] = 0xFF;  // Not used
                    can1.write(can_MsgTx);
                    break;

                case O2_SENSOR_2_B2:        // 0x19 - O2 Sensor 2 Bank 2
                    can_MsgTx.buf[0] = 0x04;
                    can_MsgTx.buf[2] = O2_SENSOR_2_B2;
                    can_MsgTx.buf[3] = 0x8D;  // 0.705V
                    can_MsgTx.buf[4] = 0xFF;  // Not used
                    can1.write(can_MsgTx);
                    break;

                case OBD_STANDARD:          // 0x1C - OBD Standard
                    can_MsgTx.buf[0] = 0x03;
                    can_MsgTx.buf[2] = OBD_STANDARD;
                    can_MsgTx.buf[3] = 0x03;  // EOBD (Europe)
                    can1.write(can_MsgTx);
                    break;

                case ENGINE_RUN_TIME:       // 0x1F - Engine run time since start
                    can_MsgTx.buf[0] = 0x04;
                    can_MsgTx.buf[2] = ENGINE_RUN_TIME;
                    can_MsgTx.buf[3] = 0x19;  // 6459 seconds
                    can_MsgTx.buf[4] = 0x3B;
                    can1.write(can_MsgTx);
                    break;
              }//switch
          }
       }
    }
   return 0;
}
     
freeze_frame_t freeze_frame[2];  // Global freeze frame storage
isotp_transfer_t isotp_tx;        // Global ISO-TP transmit context
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
}
