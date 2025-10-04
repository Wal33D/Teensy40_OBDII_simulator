/*
 * OBD-II Mode 04 - Clear/Reset Emissions Diagnostic Information
 *
 * This mode clears all emissions-related diagnostic information including:
 * - Diagnostic Trouble Codes (DTCs)
 * - Freeze Frame data
 * - Malfunction Indicator Lamp (MIL) status
 * - Number of DTCs
 * - Test results for emissions monitoring systems
 *
 * Important: Clearing DTCs does not fix the underlying problem. If the fault
 * condition still exists, the MIL will re-illuminate and DTCs will be set
 * again after the next drive cycle.
 *
 * This is mandated by SAE J1979 standard for emissions diagnostics.
 */

#include "../mode_registry.h"
#include <FlexCAN_T4.h>

// External CAN bus instance
extern FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

// External ECU data structures
extern ecu_t ecu;
extern freeze_frame_t freeze_frame[2];

/*
 * Mode 04 Handler - Clear Diagnostic Information
 *
 * Clears all emissions diagnostic data and turns off the MIL.
 * This is a critical function for emissions compliance and testing.
 *
 * According to OBD-II standards, Mode 04 shall:
 * - Clear all DTCs (stored and pending)
 * - Clear freeze frame data
 * - Turn off MIL (Check Engine Light)
 * - Reset number of DTCs to zero
 * - Clear test results for continuous and non-continuous monitors
 */
bool handle_mode_04(CAN_message_t& can_MsgRx, CAN_message_t& can_MsgTx, ecu_simClass* ecu_sim) {
    // Check if this is a Mode 04 request
    if (can_MsgRx.buf[1] != MODE4) {
        return false;  // Not our mode, let other handlers try
    }

    // Clear all diagnostic trouble codes
    ecu.dtc = false;

    // Turn off the Malfunction Indicator Lamp (MIL/Check Engine Light)
    digitalWrite(LED_red, LOW);

    // Clear freeze frame data for all stored frames
    // Freeze frames capture the vehicle's operating conditions when a DTC was set
    freeze_frame[0].data_stored = false;
    freeze_frame[1].data_stored = false;

    // Prepare positive response to Mode 04 request
    // Per SAE J1979, the response has no additional data beyond the mode echo
    can_MsgTx.buf[0] = 0x01;           // Length: 1 byte of data (just the response mode)
    can_MsgTx.buf[1] = MODE4_RESPONSE;  // 0x44 - Mode 04 positive response
    can_MsgTx.buf[2] = 0x00;           // Padding
    can_MsgTx.buf[3] = 0x00;           // Padding
    can_MsgTx.buf[4] = 0x00;           // Padding
    can_MsgTx.buf[5] = 0x00;           // Padding
    can_MsgTx.buf[6] = 0x00;           // Padding
    can_MsgTx.buf[7] = 0x00;           // Padding
    can_MsgTx.id = PID_REPLY;          // 0x7E8 - Engine ECU response
    can_MsgTx.len = 8;

    // Send the response
    can1.write(can_MsgTx);

    return true;  // Mode 04 handled the request
}

// Register Mode 04 handler at compile time
static ModeRegistrar mode_04_registrar(MODE4, handle_mode_04, "Clear Emissions Diagnostic Info");
