/*
 * OBD-II Mode 02 - Freeze Frame Data
 *
 * EMISSIONS MONITORING PURPOSE:
 * Mode 02 provides access to "freeze frame" data - a snapshot of critical
 * emissions-related sensor values captured at the moment a Diagnostic Trouble
 * Code (DTC) is triggered. This is mandated by OBD-II regulations to help
 * diagnose intermittent emissions failures.
 *
 * FREEZE FRAME OVERVIEW:
 * When an emissions-related fault occurs (e.g., misfire, sensor malfunction),
 * the ECU captures a snapshot of all relevant sensor data at that exact moment.
 * This includes:
 * - Engine RPM and vehicle speed
 * - Engine coolant temperature
 * - Throttle position
 * - Mass airflow (MAF)
 * - Oxygen sensor voltage
 *
 * This historical data helps technicians understand the operating conditions
 * when the fault occurred, making it easier to diagnose intermittent problems
 * that may not be present during inspection.
 *
 * REGULATORY CONTEXT:
 * OBD-II regulations (SAE J1979, ISO 15031-5) require all vehicles to store
 * freeze frame data for the most significant emissions-related DTCs. This
 * helps ensure proper diagnosis and repair of emissions control systems.
 */

#include "../mode_registry.h"
#include <FlexCAN_T4.h>

// External CAN bus instance
extern FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

// External ECU data structures
extern ecu_t ecu;
extern freeze_frame_t freeze_frame[2];

/*
 * Mode 02 Handler - Freeze Frame Data
 *
 * Retrieves stored freeze frame data captured when a DTC was set.
 * Unlike Mode 01 which returns current/live data, Mode 02 returns
 * historical snapshots from the moment an emissions fault occurred.
 *
 * Request Format:
 *   buf[0] = Number of data bytes
 *   buf[1] = MODE2 (0x02)
 *   buf[2] = PID requested
 *   buf[3] = Frame number (optional, defaults to 0x00)
 *
 * Response Format:
 *   buf[0] = Number of response bytes
 *   buf[1] = MODE2_RESPONSE (0x42)
 *   buf[2] = PID
 *   buf[3+] = Data bytes (from freeze frame storage)
 *
 * Frame Numbers:
 *   0x00 = Freeze frame for first DTC
 *   0x01 = Freeze frame for second DTC
 *   etc.
 *
 * This implementation supports 2 freeze frames corresponding to 2 DTCs.
 */
bool handle_mode_02(CAN_message_t& can_MsgRx, CAN_message_t& can_MsgTx, ecu_simClass* ecu_sim) {
    // Check if this is a Mode 02 request
    if (can_MsgRx.buf[1] != MODE2) {
        return false;  // Not our mode, let other handlers try
    }

    // Setup response message
    can_MsgTx.id = PID_REPLY;
    can_MsgTx.len = 8;
    can_MsgTx.buf[1] = MODE2_RESPONSE;

    // Extract frame number from request
    // Standard OBD-II: buf[2]=PID, buf[3]=Frame (optional, defaults to 0x00)
    // Check buf[0] for actual data length, or use buf[3] if provided
    uint8_t frame_num = (can_MsgRx.buf[0] >= 3) ? can_MsgRx.buf[3] : 0x00;

    // Validate frame number and check if freeze frame data exists
    // Support frame 0 and 1 (we have 2 DTCs with freeze frames)
    if (frame_num <= 1 && freeze_frame[frame_num].data_stored) {
        // Freeze frame data is available for this frame
        // Use stored freeze frame data, NOT current sensor values
        // This is critical - freeze frames are historical snapshots

        switch(can_MsgRx.buf[2])  // PID is in buf[2]
        {
            case PID_SUPPORTED:  // 0x00 - Supported PIDs in freeze frames
                // Return same supported PIDs as Mode 1
                can_MsgTx.buf[0] = 0x06;
                can_MsgTx.buf[2] = PID_SUPPORTED;
                can_MsgTx.buf[3] = 0xE8;  // Same PIDs as Mode 1
                can_MsgTx.buf[4] = 0x19;
                can_MsgTx.buf[5] = 0x30;
                can_MsgTx.buf[6] = 0x12;
                can1.write(can_MsgTx);
                break;

            case ENGINE_RPM:  // 0x0C - Engine RPM at time of DTC
                // Format: 2 bytes, formula = (A*256 + B) / 4
                can_MsgTx.buf[0] = 0x04;
                can_MsgTx.buf[2] = ENGINE_RPM;
                can_MsgTx.buf[3] = (freeze_frame[frame_num].engine_rpm & 0xff00) >> 8;
                can_MsgTx.buf[4] = freeze_frame[frame_num].engine_rpm & 0x00ff;
                can1.write(can_MsgTx);
                break;

            case ENGINE_COOLANT_TEMP:  // 0x05 - Coolant temperature at time of DTC
                // Format: 1 byte, formula = A - 40 (degrees Celsius)
                can_MsgTx.buf[0] = 0x03;
                can_MsgTx.buf[2] = ENGINE_COOLANT_TEMP;
                can_MsgTx.buf[3] = freeze_frame[frame_num].coolant_temp;
                can1.write(can_MsgTx);
                break;

            case VEHICLE_SPEED:  // 0x0D - Vehicle speed at time of DTC
                // Format: 1 byte, formula = A (km/h)
                can_MsgTx.buf[0] = 0x03;
                can_MsgTx.buf[2] = VEHICLE_SPEED;
                can_MsgTx.buf[3] = freeze_frame[frame_num].vehicle_speed;
                can1.write(can_MsgTx);
                break;

            case MAF_SENSOR:  // 0x10 - Mass airflow at time of DTC
                // Format: 2 bytes, formula = (A*256 + B) / 100 (grams/sec)
                can_MsgTx.buf[0] = 0x04;
                can_MsgTx.buf[2] = MAF_SENSOR;
                can_MsgTx.buf[3] = (freeze_frame[frame_num].maf_airflow & 0xff00) >> 8;
                can_MsgTx.buf[4] = freeze_frame[frame_num].maf_airflow & 0x00ff;
                can1.write(can_MsgTx);
                break;

            case O2_VOLTAGE:  // 0x14 - O2 sensor voltage at time of DTC
                // Format: 2 bytes
                // A = voltage (0-255), formula = A * 0.005 volts
                // B = short term fuel trim (not used here, 0xFF = N/A)
                can_MsgTx.buf[0] = 0x04;
                can_MsgTx.buf[2] = O2_VOLTAGE;
                can_MsgTx.buf[3] = freeze_frame[frame_num].o2_voltage & 0x00ff;
                can_MsgTx.buf[4] = (freeze_frame[frame_num].o2_voltage & 0xff00) >> 8;
                can1.write(can_MsgTx);
                break;

            case THROTTLE:  // 0x11 - Throttle position at time of DTC
                // Format: 1 byte, formula = A * 100 / 255 (percent)
                can_MsgTx.buf[0] = 0x03;
                can_MsgTx.buf[2] = THROTTLE;
                can_MsgTx.buf[3] = freeze_frame[frame_num].throttle_position;
                can1.write(can_MsgTx);
                break;

            default:
                // PID not supported in freeze frames
                // Return empty response
                can_MsgTx.buf[0] = 0x00;
                can1.write(can_MsgTx);
                break;
        }
    }
    else
    {
        // No freeze frame data stored for this frame number
        // Either frame number is out of range or no DTC has been set
        // Return empty response per OBD-II standard
        can_MsgTx.buf[0] = 0x00;  // No data available
        can1.write(can_MsgTx);
    }

    return true;  // Mode 02 handled the request
}

// Register Mode 02 handler at compile time
// This automatic registration makes the mode available to the ECU simulator
static ModeRegistrar mode_02_registrar(MODE2, handle_mode_02, "Freeze Frame Data");
