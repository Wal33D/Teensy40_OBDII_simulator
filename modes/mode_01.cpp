/*
 * OBD-II Mode 01 - Request Current Powertrain Data
 *
 * This mode provides access to current LIVE emissions-related data values.
 * All data must be actual readings, not default/substitute values.
 *
 * Implements realistic driving simulation with dynamic values for:
 * - Engine RPM, speed, load, throttle
 * - O2 sensors with rich/lean cycling
 * - Multiple ECU responses for scanner detection
 */

#include "../mode_registry.h"
#include <FlexCAN_T4.h>

// External CAN bus instance
extern FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

// External ECU data structures
extern ecu_t ecu;
extern freeze_frame_t freeze_frame[2];

/*
 * Mode 01 Handler - Current Powertrain Data
 *
 * Handles all Mode 01 PID requests with realistic, dynamic emissions data.
 * Simulates multiple ECUs (engine, transmission) responding appropriately.
 */
bool handle_mode_01(CAN_message_t& can_MsgRx, CAN_message_t& can_MsgTx, ecu_simClass* ecu_sim) {
    // Check if this is a Mode 01 request
    if (can_MsgRx.buf[1] != MODE1) {
        return false;  // Not our mode, let other handlers try
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
        // Formula: Voltage = A × 0.005V per SAE J1979
        // Target: 0.35V-0.55V (70-110 decimal) for proper closed-loop operation
        o2_voltage = 0x46 + random(0, 0x28);  // 0x46=70, range 40 = 70-110 = 0.35V-0.55V

        lastUpdate = millis();
    }

    switch(can_MsgRx.buf[2])  // PID is in buf[2]
    {
        case PID_SUPPORTED:  // 0x00 - PIDs 01-20
            if(sendEngineResponse) {
                can_MsgTx.id = PID_REPLY_ENGINE;
                can_MsgTx.buf[0] = 0x06;
                can_MsgTx.buf[2] = PID_SUPPORTED;
                // Bitmask showing supported PIDs 01-20
                // 0xBF = 01,03-09 | 0xBE = 0B-10 | 0xB8 = 11,13-15 | 0x93 = 19,1C,1F + PID 20 supported
                can_MsgTx.buf[3] = 0xBF;  // PIDs 01,03,04,05,06,07,08,09
                can_MsgTx.buf[4] = 0xBE;  // PIDs 0B,0C,0D,0E,0F,10
                can_MsgTx.buf[5] = 0xB8;  // PIDs 11,13,14,15 (added PID 14 support)
                can_MsgTx.buf[6] = 0x93;  // PIDs 19,1C,1F, PID 20
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
            if(ecu.dtc == 1) can_MsgTx.buf[3] = 0x82;  // MIL ON (bit 7 set) if DTC present
                else can_MsgTx.buf[3] = 0x00;          // MIL OFF if no DTC
            can_MsgTx.buf[4] = 0x07;  // From Mercedes: 0007E500
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
            can_MsgTx.id = PID_REPLY_ENGINE;
            can_MsgTx.buf[0] = 0x03;
            can_MsgTx.buf[2] = SHORT_FUEL_TRIM_2;
            can_MsgTx.buf[3] = 0x7F;  // From Mercedes: -0.8%
            can1.write(can_MsgTx);
            break;

        case LONG_FUEL_TRIM_2:  // 0x09
            can_MsgTx.id = PID_REPLY_ENGINE;
            can_MsgTx.buf[0] = 0x03;
            can_MsgTx.buf[2] = LONG_FUEL_TRIM_2;
            can_MsgTx.buf[3] = 0x7B;  // From Mercedes: -3.9%
            can1.write(can_MsgTx);
            break;

        case INTAKE_PRESSURE:  // 0x0B
            can_MsgTx.id = PID_REPLY_ENGINE;
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
            can_MsgTx.id = PID_REPLY_ENGINE;
            can_MsgTx.buf[0] = 0x03;
            can_MsgTx.buf[2] = TIMING_ADVANCE;
            can_MsgTx.buf[3] = 0x8C;  // From Mercedes: 6.0°
            can1.write(can_MsgTx);
            break;

        case INTAKE_AIR_TEMP:  // 0x0F
            can_MsgTx.id = PID_REPLY_ENGINE;
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
            can_MsgTx.id = PID_REPLY_ENGINE;
            can_MsgTx.buf[0] = 0x03;
            can_MsgTx.buf[2] = O2_SENSORS_PRESENT;
            can_MsgTx.buf[3] = 0x33;  // From Mercedes
            can1.write(can_MsgTx);
            break;

        case O2_VOLTAGE:  // 0x14 - Oxygen Sensor 1 Bank 1 (simple voltage)
            can_MsgTx.id = PID_REPLY_ENGINE;
            can_MsgTx.buf[0] = 0x04;
            can_MsgTx.buf[2] = O2_VOLTAGE;
            can_MsgTx.buf[3] = o2_voltage;  // Dynamic O2 voltage (0.35-0.55V)
            can_MsgTx.buf[4] = 0xFF;        // STFT not used in this PID format
            can1.write(can_MsgTx);
            break;

        case O2_SENSOR_2_B1:  // 0x15
            can_MsgTx.id = PID_REPLY_ENGINE;
            can_MsgTx.buf[0] = 0x04;
            can_MsgTx.buf[2] = O2_SENSOR_2_B1;
            can_MsgTx.buf[3] = o2_voltage;  // Dynamic O2 voltage
            can_MsgTx.buf[4] = 0xFF;        // Not used for trim
            can1.write(can_MsgTx);
            break;

        case O2_SENSOR_2_B2:  // 0x19
            can_MsgTx.id = PID_REPLY_ENGINE;
            can_MsgTx.buf[0] = 0x04;
            can_MsgTx.buf[2] = O2_SENSOR_2_B2;
            can_MsgTx.buf[3] = o2_voltage + 5;  // Slightly different for Bank 2
            can_MsgTx.buf[4] = 0xFF;            // Not used for trim
            can1.write(can_MsgTx);
            break;

        case OBD_STANDARD:  // 0x1C
            can_MsgTx.id = PID_REPLY_ENGINE;
            can_MsgTx.buf[0] = 0x03;
            can_MsgTx.buf[2] = OBD_STANDARD;
            can_MsgTx.buf[3] = 0x03;  // From Mercedes
            can1.write(can_MsgTx);
            break;

        case ENGINE_RUN_TIME:  // 0x1F
            can_MsgTx.id = PID_REPLY_ENGINE;
            can_MsgTx.buf[0] = 0x04;
            can_MsgTx.buf[2] = ENGINE_RUN_TIME;
            can_MsgTx.buf[3] = 0x2A;  // From Mercedes: 10926 sec
            can_MsgTx.buf[4] = 0xAE;
            can1.write(can_MsgTx);
            break;

        case DISTANCE_WITH_MIL:  // 0x21
            can_MsgTx.id = PID_REPLY_ENGINE;
            can_MsgTx.buf[0] = 0x04;
            can_MsgTx.buf[2] = DISTANCE_WITH_MIL;
            can_MsgTx.buf[3] = 0x00;  // From Mercedes: 0 km
            can_MsgTx.buf[4] = 0x00;
            can1.write(can_MsgTx);
            break;

        case FUEL_RAIL_PRESSURE:  // 0x23
            can_MsgTx.id = PID_REPLY_ENGINE;
            can_MsgTx.buf[0] = 0x04;
            can_MsgTx.buf[2] = FUEL_RAIL_PRESSURE;
            // Formula: ((A×256) + B) × 10 kPa per SAE J1979
            // Target: ~400 kPa (typical gasoline direct injection)
            can_MsgTx.buf[3] = 0x00;  // 40 decimal = 400 kPa (realistic for GDI)
            can_MsgTx.buf[4] = 0x28;  // 0x0028 = 40 × 10 = 400 kPa
            can1.write(can_MsgTx);
            break;

        case EVAP_PURGE:  // 0x2E
            can_MsgTx.id = PID_REPLY_ENGINE;
            can_MsgTx.buf[0] = 0x03;
            can_MsgTx.buf[2] = EVAP_PURGE;
            can_MsgTx.buf[3] = 0x79;  // From Mercedes: 47.5%
            can1.write(can_MsgTx);
            break;

        case FUEL_LEVEL:  // 0x2F
            can_MsgTx.id = PID_REPLY_ENGINE;
            can_MsgTx.buf[0] = 0x03;
            can_MsgTx.buf[2] = FUEL_LEVEL;
            can_MsgTx.buf[3] = 0x39;  // From Mercedes: 22.4%
            can1.write(can_MsgTx);
            break;

        case WARM_UPS:  // 0x30
            can_MsgTx.id = PID_REPLY_ENGINE;
            can_MsgTx.buf[0] = 0x03;
            can_MsgTx.buf[2] = WARM_UPS;
            can_MsgTx.buf[3] = 0xFF;  // From Mercedes
            can1.write(can_MsgTx);
            break;

        case DISTANCE_SINCE_CLR:  // 0x31
            can_MsgTx.id = PID_REPLY_ENGINE;
            can_MsgTx.buf[0] = 0x04;
            can_MsgTx.buf[2] = DISTANCE_SINCE_CLR;
            can_MsgTx.buf[3] = 0xFF;  // From Mercedes: 65535 km
            can_MsgTx.buf[4] = 0xFF;
            can1.write(can_MsgTx);
            break;

        case EVAP_VAPOR_PRESS:  // 0x32
            can_MsgTx.id = PID_REPLY_ENGINE;
            can_MsgTx.buf[0] = 0x04;
            can_MsgTx.buf[2] = EVAP_VAPOR_PRESS;
            can_MsgTx.buf[3] = 0xFD;  // From Mercedes
            can_MsgTx.buf[4] = 0xDD;
            can1.write(can_MsgTx);
            break;

        case BAROMETRIC_PRESS:  // 0x33
            can_MsgTx.id = PID_REPLY_ENGINE;
            can_MsgTx.buf[0] = 0x03;
            can_MsgTx.buf[2] = BAROMETRIC_PRESS;
            can_MsgTx.buf[3] = 0x62;  // From Mercedes: 98 kPa
            can1.write(can_MsgTx);
            break;

        case O2_SENSOR_1_B1:  // 0x34
            can_MsgTx.id = PID_REPLY_ENGINE;
            can_MsgTx.buf[0] = 0x06;
            can_MsgTx.buf[2] = O2_SENSOR_1_B1;
            can_MsgTx.buf[3] = 0x80;  // From Mercedes
            can_MsgTx.buf[4] = 0xA7;
            can_MsgTx.buf[5] = 0x80;
            can_MsgTx.buf[6] = 0x00;
            can1.write(can_MsgTx);
            break;

        case O2_SENSOR_5_B2:  // 0x38
            can_MsgTx.id = PID_REPLY_ENGINE;
            can_MsgTx.buf[0] = 0x06;
            can_MsgTx.buf[2] = O2_SENSOR_5_B2;
            can_MsgTx.buf[3] = 0x80;  // From Mercedes
            can_MsgTx.buf[4] = 0x37;
            can_MsgTx.buf[5] = 0x7F;
            can_MsgTx.buf[6] = 0xFD;
            can1.write(can_MsgTx);
            break;

        case CAT_TEMP_B1S1:  // 0x3C
            can_MsgTx.id = PID_REPLY_ENGINE;
            can_MsgTx.buf[0] = 0x04;
            can_MsgTx.buf[2] = CAT_TEMP_B1S1;
            can_MsgTx.buf[3] = 0x11;  // From Mercedes
            can_MsgTx.buf[4] = 0x7F;
            can1.write(can_MsgTx);
            break;

        case CAT_TEMP_B2S1:  // 0x3D
            can_MsgTx.id = PID_REPLY_ENGINE;
            can_MsgTx.buf[0] = 0x04;
            can_MsgTx.buf[2] = CAT_TEMP_B2S1;
            can_MsgTx.buf[3] = 0x11;  // From Mercedes
            can_MsgTx.buf[4] = 0x7E;
            can1.write(can_MsgTx);
            break;

        case MONITOR_STATUS_CYC:  // 0x41
            can_MsgTx.id = PID_REPLY_ENGINE;
            can_MsgTx.buf[0] = 0x06;
            can_MsgTx.buf[2] = MONITOR_STATUS_CYC;
            can_MsgTx.buf[3] = 0x00;  // From Mercedes
            can_MsgTx.buf[4] = 0x05;
            can_MsgTx.buf[5] = 0xE0;
            can_MsgTx.buf[6] = 0x24;
            can1.write(can_MsgTx);
            break;

        case CONTROL_MOD_VOLT:  // 0x42
            can_MsgTx.id = PID_REPLY_ENGINE;
            can_MsgTx.buf[0] = 0x04;
            can_MsgTx.buf[2] = CONTROL_MOD_VOLT;
            can_MsgTx.buf[3] = 0x33;  // From Mercedes: 13.31V
            can_MsgTx.buf[4] = 0xFF;
            can1.write(can_MsgTx);
            break;

        case ABSOLUTE_LOAD:  // 0x43
            can_MsgTx.id = PID_REPLY_ENGINE;
            can_MsgTx.buf[0] = 0x04;
            can_MsgTx.buf[2] = ABSOLUTE_LOAD;
            can_MsgTx.buf[3] = 0x00;  // From Mercedes: 17.6%
            can_MsgTx.buf[4] = 0x2D;
            can1.write(can_MsgTx);
            break;

        case COMMANDED_EQUIV:  // 0x44
            can_MsgTx.id = PID_REPLY_ENGINE;
            can_MsgTx.buf[0] = 0x04;
            can_MsgTx.buf[2] = COMMANDED_EQUIV;
            can_MsgTx.buf[3] = 0x7F;  // From Mercedes
            can_MsgTx.buf[4] = 0xFF;
            can1.write(can_MsgTx);
            break;

        case REL_THROTTLE_POS:  // 0x45
            can_MsgTx.id = PID_REPLY_ENGINE;
            can_MsgTx.buf[0] = 0x03;
            can_MsgTx.buf[2] = REL_THROTTLE_POS;
            can_MsgTx.buf[3] = currentThrottle >> 2;  // Relative throttle (1/4 of absolute)
            can1.write(can_MsgTx);
            break;

        case AMBIENT_AIR_TEMP:  // 0x46
            can_MsgTx.id = PID_REPLY_ENGINE;
            can_MsgTx.buf[0] = 0x03;
            can_MsgTx.buf[2] = AMBIENT_AIR_TEMP;
            can_MsgTx.buf[3] = 0x4E;  // From Mercedes: 38°C
            can1.write(can_MsgTx);
            break;

        case THROTTLE_POS_B:  // 0x47
            can_MsgTx.id = PID_REPLY_ENGINE;
            can_MsgTx.buf[0] = 0x03;
            can_MsgTx.buf[2] = THROTTLE_POS_B;
            can_MsgTx.buf[3] = currentThrottle;  // Same as throttle A
            can1.write(can_MsgTx);
            break;

        case ACCEL_POS_D:  // 0x49
            can_MsgTx.id = PID_REPLY_ENGINE;
            can_MsgTx.buf[0] = 0x03;
            can_MsgTx.buf[2] = ACCEL_POS_D;
            can_MsgTx.buf[3] = 0x11;  // From Mercedes
            can1.write(can_MsgTx);
            break;

        case ACCEL_POS_E:  // 0x4A
            can_MsgTx.id = PID_REPLY_ENGINE;
            can_MsgTx.buf[0] = 0x03;
            can_MsgTx.buf[2] = ACCEL_POS_E;
            can_MsgTx.buf[3] = 0x11;  // From Mercedes
            can1.write(can_MsgTx);
            break;

        case COMMANDED_THROTTLE:  // 0x4C
            can_MsgTx.id = PID_REPLY_ENGINE;
            can_MsgTx.buf[0] = 0x03;
            can_MsgTx.buf[2] = COMMANDED_THROTTLE;
            can_MsgTx.buf[3] = currentThrottle >> 1;  // Half of actual throttle
            can1.write(can_MsgTx);
            break;

        case FUEL_TYPE:  // 0x51
            can_MsgTx.id = PID_REPLY_ENGINE;
            can_MsgTx.buf[0] = 0x03;
            can_MsgTx.buf[2] = FUEL_TYPE;
            can_MsgTx.buf[3] = 0x01;  // From Mercedes
            can1.write(can_MsgTx);
            break;

        case SHORT_O2_TRIM_B1:  // 0x56
            can_MsgTx.id = PID_REPLY_ENGINE;
            can_MsgTx.buf[0] = 0x03;
            can_MsgTx.buf[2] = SHORT_O2_TRIM_B1;
            can_MsgTx.buf[3] = 0x7E;  // From Mercedes
            can1.write(can_MsgTx);
            break;

        case SHORT_O2_TRIM_B2:  // 0x58
            can_MsgTx.id = PID_REPLY_ENGINE;
            can_MsgTx.buf[0] = 0x03;
            can_MsgTx.buf[2] = SHORT_O2_TRIM_B2;
            can_MsgTx.buf[3] = 0x7F;  // From Mercedes
            can1.write(can_MsgTx);
            break;

        default:
            // Send negative response for unsupported PIDs (7F response)
            can_MsgTx.id = PID_REPLY_ENGINE;
            can_MsgTx.len = 8;
            can_MsgTx.buf[0] = 0x03;  // Length: 3 bytes
            can_MsgTx.buf[1] = 0x7F;  // Negative Response Service Identifier
            can_MsgTx.buf[2] = 0x01;  // Echo requested service (Mode 1)
            can_MsgTx.buf[3] = can_MsgRx.buf[2];  // Echo requested PID
            can_MsgTx.buf[4] = 0x12;  // NRC: requestSequenceError (PID not supported)
            can_MsgTx.buf[5] = 0x00;  // Padding
            can_MsgTx.buf[6] = 0x00;  // Padding
            can_MsgTx.buf[7] = 0x00;  // Padding
            can1.write(can_MsgTx);
            break;
    }

    return true;  // Mode 01 handled the request
}

// Register Mode 01 handler at compile time
static ModeRegistrar mode_01_registrar(MODE1, handle_mode_01, "Current Powertrain Data");
