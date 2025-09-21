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

#include <Bounce2.h>
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
          // Activate DTCs - simulating 2 emission faults
          ecu.dtc = 1;  // Flag indicating DTCs are active
          digitalWrite(LED_red, HIGH);  // Turn on MIL (Malfunction Indicator Lamp)

          // OBD-II Freeze Frame Capture per SAE J1979
          // Snapshot of operating conditions when DTC is set
          // We store 2 DTCs with their freeze frames to match Mode 01 PID 01 reporting

          // Frame 0x00: Primary freeze frame (highest priority DTC)
          freeze_frame[0].coolant_temp = ecu.coolant_temp;
          freeze_frame[0].engine_rpm = ecu.engine_rpm;
          freeze_frame[0].throttle_position = ecu.throttle_position;
          freeze_frame[0].vehicle_speed = ecu.vehicle_speed;
          freeze_frame[0].maf_airflow = ecu.maf_airflow;
          freeze_frame[0].o2_voltage = ecu.o2_voltage;
          freeze_frame[0].data_stored = true;
          freeze_frame[0].dtc_code = 0x0100;  // P0100: MAF Circuit Malfunction

          // Frame 0x01: Secondary freeze frame
          freeze_frame[1].coolant_temp = ecu.coolant_temp;
          freeze_frame[1].engine_rpm = ecu.engine_rpm;
          freeze_frame[1].throttle_position = ecu.throttle_position;
          freeze_frame[1].vehicle_speed = ecu.vehicle_speed;
          freeze_frame[1].maf_airflow = ecu.maf_airflow;
          freeze_frame[1].o2_voltage = ecu.o2_voltage;
          freeze_frame[1].data_stored = true;
          freeze_frame[1].dtc_code = 0x0200;  // P0200: Injector Circuit Malfunction

      }else
      {
          // Clear all DTCs and turn off MIL
          ecu.dtc = 0;
          digitalWrite(LED_red, LOW);

          // Also clear freeze frames when manually clearing via button
          freeze_frame[0].data_stored = false;
          freeze_frame[1].data_stored = false;
      }
    }
  }
}


uint8_t ecu_simClass::update(void) 
{
  CAN_message_t can_MsgRx,can_MsgTx;

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
     
     if (can_MsgRx.id == PID_REQUEST) 
     {
       digitalWrite(LED_green, HIGH);
       flash_led_tick = 0;

        if(can_MsgRx.buf[1] == MODE3) // Request trouble codes
        {
            // Real vehicles: Each ECU reports its own DTCs
            // Engine ECU (0x7E8): P0xxx codes
            // Trans ECU (0x7E9): P07xx codes
            // Body ECU (0x7EA): B/U codes

            if(ecu.dtc == false){
                // No DTCs - only primary ECU responds with "no codes"
                can_MsgTx.id = PID_REPLY;
                can_MsgTx.buf[0] = 0x02;
                can_MsgTx.buf[1] = MODE3_RESPONSE;
                can_MsgTx.buf[2] = 0x00;  // 0 DTCs
                can_MsgTx.len = 8;
                can1.write(can_MsgTx);
            }else{
                // Engine ECU reports engine DTCs
                can_MsgTx.id = 0x7E8;
                can_MsgTx.buf[0] = 0x04;
                can_MsgTx.buf[1] = MODE3_RESPONSE;
                can_MsgTx.buf[2] = 0x01;  // 1 DTC from engine
                can_MsgTx.buf[3] = 0x01;  // P0100
                can_MsgTx.buf[4] = 0x00;  // MAF Circuit
                can_MsgTx.len = 8;
                can1.write(can_MsgTx);

                // Variable delay for bus arbitration
                delay(random(5, 12));

                // Transmission ECU reports trans DTCs
                can_MsgTx.id = 0x7E9;
                can_MsgTx.buf[0] = 0x04;
                can_MsgTx.buf[1] = MODE3_RESPONSE;
                can_MsgTx.buf[2] = 0x01;  // 1 DTC from trans
                can_MsgTx.buf[3] = 0x07;  // P0700
                can_MsgTx.buf[4] = 0x00;  // Trans Control System
                can_MsgTx.len = 8;
                can1.write(can_MsgTx);
            }
        }
      
        if(can_MsgRx.buf[1] == MODE4) // Clear trouble codes, clear Check engine light
        {
            // Clear all DTCs - this will update Mode 01 PID 01 to report 0 DTCs
            ecu.dtc = false;
            digitalWrite(LED_red, LOW);  // Turn off MIL (Check Engine Light)

            // Clear freeze frame data per OBD-II standard
            freeze_frame[0].data_stored = false;
            freeze_frame[1].data_stored = false;

            // Send Mode 04 acknowledgment
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

            // OBD-II Standard: Mode 02 request format is always:
            // buf[0]=length, buf[1]=0x02, buf[2]=PID, buf[3]=Frame Number
            // Frame number is ALWAYS in buf[3] for Mode 02
            uint8_t frame_num = can_MsgRx.buf[3];

            // Validate frame number (we support frames 0x00 and 0x01)
            if(frame_num > 1) {
                // Invalid frame number - respond with no data
                can_MsgTx.buf[0] = 0x00;
                can1.write(can_MsgTx);
                return 0;
            }

            // Check if requested frame has stored data
            if(freeze_frame[frame_num].data_stored)
            {
                // Use stored freeze frame data, not current sensor values
                switch(can_MsgRx.buf[2])  // PID is in buf[2]
                {
                    case PID_SUPPORTED:
                        can_MsgTx.buf[0] = 0x06;
                        can_MsgTx.buf[2] = PID_SUPPORTED;
                        can_MsgTx.buf[3] = frame_num;     // Include frame number
                        can_MsgTx.buf[4] = 0xEC;          // Supported PIDs including 0x02 (bit 2 set)
                        can_MsgTx.buf[5] = 0x19;
                        can_MsgTx.buf[6] = 0x30;
                        can1.write(can_MsgTx);
                        break;

                    case DTC_FREEZE_FRAME:  // PID 0x02 - DTC that caused freeze frame
                        can_MsgTx.buf[0] = 0x05;  // 5 bytes: SID + PID + Frame + 2-byte DTC
                        can_MsgTx.buf[2] = DTC_FREEZE_FRAME;
                        can_MsgTx.buf[3] = frame_num;     // Include frame number
                        can_MsgTx.buf[4] = (freeze_frame[frame_num].dtc_code & 0xFF00) >> 8;  // DTC high byte
                        can_MsgTx.buf[5] = freeze_frame[frame_num].dtc_code & 0x00FF;        // DTC low byte
                        can1.write(can_MsgTx);
                        break;

                    case ENGINE_RPM:
                        can_MsgTx.buf[0] = 0x05;
                        can_MsgTx.buf[2] = ENGINE_RPM;
                        can_MsgTx.buf[3] = frame_num;     // Include frame number
                        can_MsgTx.buf[4] = (freeze_frame[frame_num].engine_rpm & 0xff00) >> 8;
                        can_MsgTx.buf[5] = freeze_frame[frame_num].engine_rpm & 0x00ff;
                        can1.write(can_MsgTx);
                        break;

                    case ENGINE_COOLANT_TEMP:
                        can_MsgTx.buf[0] = 0x04;
                        can_MsgTx.buf[2] = ENGINE_COOLANT_TEMP;
                        can_MsgTx.buf[3] = frame_num;     // Include frame number
                        can_MsgTx.buf[4] = freeze_frame[frame_num].coolant_temp;
                        can1.write(can_MsgTx);
                        break;

                    case VEHICLE_SPEED:
                        can_MsgTx.buf[0] = 0x04;
                        can_MsgTx.buf[2] = VEHICLE_SPEED;
                        can_MsgTx.buf[3] = frame_num;     // Include frame number
                        can_MsgTx.buf[4] = freeze_frame[frame_num].vehicle_speed;
                        can1.write(can_MsgTx);
                        break;

                    case MAF_SENSOR:
                        can_MsgTx.buf[0] = 0x05;
                        can_MsgTx.buf[2] = MAF_SENSOR;
                        can_MsgTx.buf[3] = frame_num;     // Include frame number
                        can_MsgTx.buf[4] = (freeze_frame[frame_num].maf_airflow & 0xff00) >> 8;
                        can_MsgTx.buf[5] = freeze_frame[frame_num].maf_airflow & 0x00ff;
                        can1.write(can_MsgTx);
                        break;

                    case O2_VOLTAGE:
                        can_MsgTx.buf[0] = 0x05;
                        can_MsgTx.buf[2] = O2_VOLTAGE;
                        can_MsgTx.buf[3] = frame_num;     // Include frame number
                        can_MsgTx.buf[4] = freeze_frame[frame_num].o2_voltage & 0x00ff;
                        can_MsgTx.buf[5] = (freeze_frame[frame_num].o2_voltage & 0xff00) >> 8;
                        can1.write(can_MsgTx);
                        break;

                    case THROTTLE:
                        can_MsgTx.buf[0] = 0x04;
                        can_MsgTx.buf[2] = THROTTLE;
                        can_MsgTx.buf[3] = frame_num;     // Include frame number
                        can_MsgTx.buf[4] = freeze_frame[frame_num].throttle_position;
                        can1.write(can_MsgTx);
                        break;
                }
            }
            else
            {
                // No freeze frame data stored for this frame number
                // OBD-II Standard: Return appropriate response based on PID
                switch(can_MsgRx.buf[2])  // Check requested PID
                {
                    case PID_SUPPORTED:
                        // Always respond with supported PIDs even if no data
                        can_MsgTx.buf[0] = 0x06;
                        can_MsgTx.buf[2] = PID_SUPPORTED;
                        can_MsgTx.buf[3] = frame_num;
                        can_MsgTx.buf[4] = 0xEC;  // PIDs supported
                        can_MsgTx.buf[5] = 0x19;
                        can_MsgTx.buf[6] = 0x30;
                        can1.write(can_MsgTx);
                        break;

                    case DTC_FREEZE_FRAME:
                        // Return 0x0000 to indicate no freeze frame exists
                        can_MsgTx.buf[0] = 0x05;
                        can_MsgTx.buf[2] = DTC_FREEZE_FRAME;
                        can_MsgTx.buf[3] = frame_num;
                        can_MsgTx.buf[4] = 0x00;  // No DTC
                        can_MsgTx.buf[5] = 0x00;  // No DTC
                        can1.write(can_MsgTx);
                        break;

                    default:
                        // For other PIDs, return no data
                        can_MsgTx.buf[0] = 0x00;  // No data available
                        can1.write(can_MsgTx);
                        break;
                }
            }
        }

        if(can_MsgRx.buf[1] == MODE1) // Live data
        {
            // Determine which ECU should respond based on PID
            // In real vehicles, different ECUs handle different PIDs
            uint8_t pid = can_MsgRx.buf[2];
            bool engine_ecu_responds = false;
            bool trans_ecu_responds = false;
            bool body_ecu_responds = false;

            // Determine which ECUs should respond to this PID
            // Based on real vehicle behavior
            switch(pid) {
                // Engine ECU handles most engine-related PIDs
                case PID_SUPPORTED:
                case MONITOR_STATUS:
                case FUEL_SYSTEM_STATUS:
                case CALCULATED_LOAD:
                case ENGINE_COOLANT_TEMP:
                case SHORT_FUEL_TRIM_1:
                case LONG_FUEL_TRIM_1:
                case SHORT_FUEL_TRIM_2:
                case LONG_FUEL_TRIM_2:
                case INTAKE_PRESSURE:
                case ENGINE_RPM:
                case TIMING_ADVANCE:
                case INTAKE_AIR_TEMP:
                case MAF_SENSOR:
                case THROTTLE:
                case O2_SENSORS_PRESENT:
                case O2_VOLTAGE:
                case O2_SENSOR_2_B1:
                case O2_SENSOR_2_B2:
                    engine_ecu_responds = true;
                    break;

                // Transmission ECU only responds to speed and some PIDs
                case VEHICLE_SPEED:
                    engine_ecu_responds = true;
                    trans_ecu_responds = true;  // Both can provide speed
                    break;

                // Body ECU handles battery voltage
                case CONTROL_MOD_VOLT:
                    body_ecu_responds = true;
                    break;

                // All ECUs respond to supported PIDs query
                case PID_20_SUPPORTED:
                case PID_40_SUPPORTED:
                    engine_ecu_responds = true;
                    trans_ecu_responds = true;
                    body_ecu_responds = true;
                    break;
            }

            can_MsgTx.id = PID_REPLY;
            can_MsgTx.len = 8;
            can_MsgTx.buf[1] = MODE1_RESPONSE;

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
                    if(engine_ecu_responds) {
                        // Engine ECU - full capabilities
                        can_MsgTx.buf[0] = 0x06;
                        can_MsgTx.buf[2] = PID_SUPPORTED;
                        can_MsgTx.buf[3] = 0xBF;  // Engine-related PIDs
                        can_MsgTx.buf[4] = 0xBE;
                        can_MsgTx.buf[5] = 0xA8;
                        can_MsgTx.buf[6] = 0x93;
                        can1.write(can_MsgTx);
                    }

                    if(trans_ecu_responds) {
                        // Variable timing (5-15ms) for realism
                        delay(random(5, 15));

                        // Transmission ECU - limited capabilities
                        can_MsgTx.id = 0x7E9;
                        can_MsgTx.buf[0] = 0x06;
                        can_MsgTx.buf[2] = PID_SUPPORTED;
                        can_MsgTx.buf[3] = 0x00;  // Only speed (0x0D)
                        can_MsgTx.buf[4] = 0x20;  // Bit 5 = PID 0x0D
                        can_MsgTx.buf[5] = 0x00;
                        can_MsgTx.buf[6] = 0x00;
                        can1.write(can_MsgTx);
                    }

                    if(body_ecu_responds) {
                        delay(random(5, 15));

                        // Body ECU - minimal capabilities
                        can_MsgTx.id = 0x7EA;
                        can_MsgTx.buf[0] = 0x06;
                        can_MsgTx.buf[2] = PID_SUPPORTED;
                        can_MsgTx.buf[3] = 0x00;  // No PIDs in 01-20
                        can_MsgTx.buf[4] = 0x00;
                        can_MsgTx.buf[5] = 0x00;
                        can_MsgTx.buf[6] = 0x00;
                        can1.write(can_MsgTx);
                    }

                    // Reset ID for next messages
                    can_MsgTx.id = PID_REPLY;
                    break;

                case PID_20_SUPPORTED:  // 0x20 - PIDs 21-40
                    can_MsgTx.buf[0] = 0x06;
                    can_MsgTx.buf[2] = PID_20_SUPPORTED;
                    can_MsgTx.buf[3] = 0xA0;  // From Mercedes: A007F119
                    can_MsgTx.buf[4] = 0x07;
                    can_MsgTx.buf[5] = 0xF1;
                    can_MsgTx.buf[6] = 0x19;
                    can1.write(can_MsgTx);
                    break;

                case PID_40_SUPPORTED:  // 0x40 - PIDs 41-60
                    can_MsgTx.buf[0] = 0x06;
                    can_MsgTx.buf[2] = PID_40_SUPPORTED;
                    can_MsgTx.buf[3] = 0xFE;  // From Mercedes: FED08500
                    can_MsgTx.buf[4] = 0xD0;
                    can_MsgTx.buf[5] = 0x85;
                    can_MsgTx.buf[6] = 0x00;
                    can1.write(can_MsgTx);
                    break;

                case MONITOR_STATUS:  // 0x01
                    can_MsgTx.buf[0] = 0x06;
                    can_MsgTx.buf[2] = MONITOR_STATUS;
                    // OBD-II Standard: Byte 1 = MIL + DTC count
                    // Bit 7: MIL on/off, Bits 0-6: Number of DTCs
                    if(ecu.dtc == 1) {
                        can_MsgTx.buf[3] = 0x82;  // MIL on (bit 7) + 2 DTCs (P0100, P0200)
                    } else {
                        can_MsgTx.buf[3] = 0x00;  // MIL off + 0 DTCs
                    }
                    can_MsgTx.buf[4] = 0x07;  // Monitor status
                    can_MsgTx.buf[5] = 0xE5;  // Monitor status
                    can_MsgTx.buf[6] = 0x00;  // Monitor status
                    can1.write(can_MsgTx);
                    break;

                case FUEL_SYSTEM_STATUS:  // 0x03
                    can_MsgTx.buf[0] = 0x04;
                    can_MsgTx.buf[2] = FUEL_SYSTEM_STATUS;
                    can_MsgTx.buf[3] = 0x02;  // From Mercedes: 0200
                    can_MsgTx.buf[4] = 0x00;
                    can1.write(can_MsgTx);
                    break;

                case CALCULATED_LOAD:  // 0x04
                    can_MsgTx.buf[0] = 0x03;
                    can_MsgTx.buf[2] = CALCULATED_LOAD;
                    can_MsgTx.buf[3] = currentLoad;  // Dynamic load value
                    can1.write(can_MsgTx);
                    break;

                case ENGINE_COOLANT_TEMP:  // 0x05
                    can_MsgTx.buf[0] = 0x03;
                    can_MsgTx.buf[2] = ENGINE_COOLANT_TEMP;
                    can_MsgTx.buf[3] = 0x87;  // From Mercedes: 95°C (0x87)
                    can1.write(can_MsgTx);
                    break;

                case SHORT_FUEL_TRIM_1:  // 0x06
                    can_MsgTx.buf[0] = 0x03;
                    can_MsgTx.buf[2] = SHORT_FUEL_TRIM_1;
                    can_MsgTx.buf[3] = 0x7F;  // From Mercedes: -0.8%
                    can1.write(can_MsgTx);
                    break;

                case LONG_FUEL_TRIM_1:  // 0x07
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
                    can_MsgTx.buf[0] = 0x04;
                    can_MsgTx.buf[2] = ENGINE_RPM;
                    can_MsgTx.buf[3] = (currentRPM >> 8) & 0xFF;
                    can_MsgTx.buf[4] = currentRPM & 0xFF;
                    can1.write(can_MsgTx);
                    break;

                case VEHICLE_SPEED:  // 0x0D
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
                        can_MsgTx.buf[0] = 0x04;
                        can_MsgTx.buf[2] = MAF_SENSOR;
                        can_MsgTx.buf[3] = (maf_value >> 8) & 0xFF;
                        can_MsgTx.buf[4] = maf_value & 0xFF;
                        can1.write(can_MsgTx);
                    }
                    break;

                case THROTTLE:  // 0x11
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
                    // Unknown PID - no response
                    break;
            }
        }

        if(can_MsgRx.buf[1] == MODE9) // Vehicle information
        {
            can_MsgTx.id = PID_REPLY;
            can_MsgTx.len = 8;
            can_MsgTx.buf[1] = MODE9_RESPONSE;

            switch(can_MsgRx.buf[2])  // PID is in buf[2]
            {
                case VEH_INFO_SUPPORTED:  // 0x00 - Supported PIDs
                    // Multi-ECU response like real Mercedes
                    // Primary ECU has most info
                    can_MsgTx.buf[0] = 0x06;  // Single frame, 6 bytes
                    can_MsgTx.buf[2] = VEH_INFO_SUPPORTED;  // PID = 0x00
                    can_MsgTx.buf[3] = 0x55;  // Bitmap: VIN(02), CAL_ID(04), CVN(06) supported
                    can_MsgTx.buf[4] = 0x40;  // Bitmap: ECU_NAME(0A) supported
                    can_MsgTx.buf[5] = 0x10;  // Bitmap: PERF_TRACK(08), AUX_IO(14) supported
                    can_MsgTx.buf[6] = 0x00;  // Bitmap: remaining PIDs
                    can1.write(can_MsgTx);

                    // Small delay like real CAN bus timing
                    delay(5);

                    // Second ECU (Transmission) - limited Mode 09 support
                    can_MsgTx.id = 0x7E9;
                    can_MsgTx.buf[0] = 0x06;
                    can_MsgTx.buf[1] = MODE9_RESPONSE;
                    can_MsgTx.buf[2] = VEH_INFO_SUPPORTED;
                    can_MsgTx.buf[3] = 0x14;  // Less support than engine ECU
                    can_MsgTx.buf[4] = 0x40;
                    can_MsgTx.buf[5] = 0x00;
                    can_MsgTx.buf[6] = 0x00;
                    can1.write(can_MsgTx);

                    // Another small delay
                    delay(5);

                    // Third ECU (Body/Chassis) - minimal Mode 09 support
                    can_MsgTx.id = 0x7EA;
                    can_MsgTx.buf[0] = 0x06;
                    can_MsgTx.buf[1] = MODE9_RESPONSE;
                    can_MsgTx.buf[2] = VEH_INFO_SUPPORTED;
                    can_MsgTx.buf[3] = 0x14;  // Minimal support
                    can_MsgTx.buf[4] = 0x40;
                    can_MsgTx.buf[5] = 0x00;
                    can_MsgTx.buf[6] = 0x00;
                    can1.write(can_MsgTx);

                    // Reset to primary ECU ID
                    can_MsgTx.id = PID_REPLY;
                    break;

                case VIN_REQUEST:  // 0x02 - Vehicle Identification Number
                    {
                    // VIN: 4JGDA5HB7JB158144 (17 chars)
                    // Note: ELM327 handles ISO-TP flow control automatically

                    // Send First Frame
                    can_MsgTx.buf[0] = 0x10;  // First frame
                    can_MsgTx.buf[1] = 0x14;  // 20 bytes total (3 header + 17 VIN)
                    can_MsgTx.buf[2] = MODE9_RESPONSE;
                    can_MsgTx.buf[3] = VIN_REQUEST;
                    can_MsgTx.buf[4] = 0x01;  // 1 data item
                    can_MsgTx.buf[5] = 0x34;  // '4' in hex
                    can_MsgTx.buf[6] = 0x4A;  // 'J' in hex
                    can_MsgTx.buf[7] = 0x47;  // 'G' in hex
                    can1.write(can_MsgTx);

                    // Small delay between frames for bus timing
                    delay(10);

                    can_MsgTx.buf[0] = 0x21;  // Consecutive frame 1
                    can_MsgTx.buf[1] = 0x44;  // ASCII 'D'
                    can_MsgTx.buf[2] = 0x41;  // ASCII 'A'
                    can_MsgTx.buf[3] = 0x35;  // ASCII '5'
                    can_MsgTx.buf[4] = 0x48;  // ASCII 'H'
                    can_MsgTx.buf[5] = 0x42;  // ASCII 'B'
                    can_MsgTx.buf[6] = 0x37;  // ASCII '7'
                    can_MsgTx.buf[7] = 0x4A;  // ASCII 'J'
                    can1.write(can_MsgTx);

                    delay(10);

                    can_MsgTx.buf[0] = 0x22;  // Consecutive frame 2
                    can_MsgTx.buf[1] = 0x42;  // ASCII 'B'
                    can_MsgTx.buf[2] = 0x31;  // ASCII '1'
                    can_MsgTx.buf[3] = 0x35;  // ASCII '5'
                    can_MsgTx.buf[4] = 0x38;  // ASCII '8'
                    can_MsgTx.buf[5] = 0x31;  // ASCII '1'
                    can_MsgTx.buf[6] = 0x34;  // ASCII '4'
                    can_MsgTx.buf[7] = 0x34;  // ASCII '4'
                    can1.write(can_MsgTx);
                    }
                    break;

                case CAL_ID_REQUEST:  // 0x04 - Calibration ID
                    // Cal ID: 2769011200190170 (16 chars)
                    // Real response: 49040132373639303131323030313930313730
                    can_MsgTx.buf[0] = 0x10;  // First frame
                    can_MsgTx.buf[1] = 0x13;  // 19 bytes total
                    can_MsgTx.buf[2] = MODE9_RESPONSE;
                    can_MsgTx.buf[3] = CAL_ID_REQUEST;
                    can_MsgTx.buf[4] = 0x01;  // 1 data item
                    can_MsgTx.buf[5] = 0x32;  // ASCII '2'
                    can_MsgTx.buf[6] = 0x37;  // ASCII '7'
                    can_MsgTx.buf[7] = 0x36;  // ASCII '6'
                    can1.write(can_MsgTx);

                    delay(10);
                    can_MsgTx.buf[0] = 0x21;  // Consecutive frame 1
                    can_MsgTx.buf[1] = 0x39;  // ASCII '9'
                    can_MsgTx.buf[2] = 0x30;  // ASCII '0'
                    can_MsgTx.buf[3] = 0x31;  // ASCII '1'
                    can_MsgTx.buf[4] = 0x31;  // ASCII '1'
                    can_MsgTx.buf[5] = 0x32;  // ASCII '2'
                    can_MsgTx.buf[6] = 0x30;  // ASCII '0'
                    can_MsgTx.buf[7] = 0x30;  // ASCII '0'
                    can1.write(can_MsgTx);

                    delay(10);
                    can_MsgTx.buf[0] = 0x22;  // Consecutive frame 2
                    can_MsgTx.buf[1] = 0x31;  // ASCII '1'
                    can_MsgTx.buf[2] = 0x39;  // ASCII '9'
                    can_MsgTx.buf[3] = 0x30;  // ASCII '0'
                    can_MsgTx.buf[4] = 0x31;  // ASCII '1'
                    can_MsgTx.buf[5] = 0x37;  // ASCII '7'
                    can_MsgTx.buf[6] = 0x30;  // ASCII '0'
                    can_MsgTx.buf[7] = 0x00;  // Padding
                    can1.write(can_MsgTx);
                    break;

                case CVN_REQUEST:  // 0x06 - Calibration Verification Number
                    // Real response: 490601EB854939
                    can_MsgTx.buf[0] = 0x07;  // Single frame, 7 bytes total
                    can_MsgTx.buf[1] = MODE9_RESPONSE;
                    can_MsgTx.buf[2] = CVN_REQUEST;
                    can_MsgTx.buf[3] = 0x01;  // 1 CVN
                    can_MsgTx.buf[4] = 0xEB;
                    can_MsgTx.buf[5] = 0x85;
                    can_MsgTx.buf[6] = 0x49;
                    can_MsgTx.buf[7] = 0x39;
                    can1.write(can_MsgTx);
                    break;

                case ECU_NAME_REQUEST:  // 0x0A - ECU Name
                    // ECU Name: "ECM-EngineControl"
                    // Real response: 490A0145434D002D456E67696E65436F6E74726F6C0000
                    can_MsgTx.buf[0] = 0x10;  // First frame
                    can_MsgTx.buf[1] = 0x17;  // 23 bytes total
                    can_MsgTx.buf[2] = MODE9_RESPONSE;
                    can_MsgTx.buf[3] = ECU_NAME_REQUEST;
                    can_MsgTx.buf[4] = 0x01;  // 1 data item
                    can_MsgTx.buf[5] = 0x45;  // ASCII 'E'
                    can_MsgTx.buf[6] = 0x43;  // ASCII 'C'
                    can_MsgTx.buf[7] = 0x4D;  // ASCII 'M'
                    can1.write(can_MsgTx);

                    delay(10);
                    can_MsgTx.buf[0] = 0x21;  // Consecutive frame 1
                    can_MsgTx.buf[1] = 0x00;  // Separator
                    can_MsgTx.buf[2] = 0x2D;  // ASCII '-'
                    can_MsgTx.buf[3] = 0x45;  // ASCII 'E'
                    can_MsgTx.buf[4] = 0x6E;  // ASCII 'n'
                    can_MsgTx.buf[5] = 0x67;  // ASCII 'g'
                    can_MsgTx.buf[6] = 0x69;  // ASCII 'i'
                    can_MsgTx.buf[7] = 0x6E;  // ASCII 'n'
                    can1.write(can_MsgTx);

                    delay(10);
                    can_MsgTx.buf[0] = 0x22;  // Consecutive frame 2
                    can_MsgTx.buf[1] = 0x65;  // ASCII 'e'
                    can_MsgTx.buf[2] = 0x43;  // ASCII 'C'
                    can_MsgTx.buf[3] = 0x6F;  // ASCII 'o'
                    can_MsgTx.buf[4] = 0x6E;  // ASCII 'n'
                    can_MsgTx.buf[5] = 0x74;  // ASCII 't'
                    can_MsgTx.buf[6] = 0x72;  // ASCII 'r'
                    can_MsgTx.buf[7] = 0x6F;  // ASCII 'o'
                    can1.write(can_MsgTx);

                    delay(10);
                    can_MsgTx.buf[0] = 0x23;  // Consecutive frame 3
                    can_MsgTx.buf[1] = 0x6C;  // ASCII 'l'
                    can_MsgTx.buf[2] = 0x00;  // Null terminator
                    can_MsgTx.buf[3] = 0x00;  // Padding
                    can_MsgTx.buf[4] = 0x00;  // Padding
                    can_MsgTx.buf[5] = 0x00;  // Padding
                    can_MsgTx.buf[6] = 0x00;  // Padding
                    can_MsgTx.buf[7] = 0x00;  // Padding
                    can1.write(can_MsgTx);
                    break;

                case PERF_TRACK_REQUEST:  // 0x08 - Performance Tracking
                    // Real vehicle sends 43 bytes (86 hex chars)
                    // ISO-TP COMPLIANT: Multi-frame sequence required

                    // First frame (FF) - ISO-TP format
                    can_MsgTx.buf[0] = 0x10;  // First frame PCI
                    can_MsgTx.buf[1] = 0x2B;  // Length = 43 bytes (0x2B)
                    can_MsgTx.buf[2] = MODE9_RESPONSE;      // 0x49
                    can_MsgTx.buf[3] = PERF_TRACK_REQUEST;  // 0x08
                    can_MsgTx.buf[4] = 0x14;  // Real data byte 1
                    can_MsgTx.buf[5] = 0x10;  // Real data byte 2
                    can_MsgTx.buf[6] = 0x62;  // Real data byte 3
                    can_MsgTx.buf[7] = 0x2E;  // Real data byte 4
                    can1.write(can_MsgTx);

                    // ISO-TP requires 10ms delay between frames
                    delay(10);

                    // Consecutive frame 1 (CF1)
                    can_MsgTx.buf[0] = 0x21;  // CF sequence 1
                    can_MsgTx.buf[1] = 0x4C;  // Continue real data
                    can_MsgTx.buf[2] = 0x17;
                    can_MsgTx.buf[3] = 0x69;
                    can_MsgTx.buf[4] = 0x10;
                    can_MsgTx.buf[5] = 0x62;
                    can_MsgTx.buf[6] = 0x17;
                    can_MsgTx.buf[7] = 0x04;
                    can1.write(can_MsgTx);

                    // Note: Full implementation would need 6 consecutive frames total
                    // Simplified for emulator - most scanners accept partial data
                    break;

                case AUX_IO_REQUEST:  // 0x14 - Auxiliary I/O Status
                    // Real response: 4914010018 (5 bytes total)
                    can_MsgTx.buf[0] = 0x05;  // 5 bytes of data
                    can_MsgTx.buf[2] = AUX_IO_REQUEST;
                    can_MsgTx.buf[3] = 0x01;  // 1 data item
                    can_MsgTx.buf[4] = 0x00;
                    can_MsgTx.buf[5] = 0x18;
                    can1.write(can_MsgTx);
                    break;
            }
        }

        /* REMOVED OLD STATIC MODE1 HANDLER - Using dynamic one above instead */
        /*
        if(can_MsgRx.buf[1] == MODE1)
        {
            can_MsgTx.id = PID_REPLY;
            can_MsgTx.len = 8;
            can_MsgTx.buf[1] = MODE1_RESPONSE;

            switch(can_MsgRx.buf[2])
            {
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
       */
       }
    }

    return 0;
}
     
freeze_frame_t freeze_frame[2];  // Global freeze frame storage
ecu_simClass ecu_sim;
