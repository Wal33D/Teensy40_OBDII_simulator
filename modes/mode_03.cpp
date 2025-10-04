/*
 * OBD-II Mode 03 - Request Emissions-Related Diagnostic Trouble Codes
 *
 * This mode provides access to stored emissions-related DTCs (Diagnostic Trouble Codes)
 * that have triggered the Malfunction Indicator Lamp (MIL/Check Engine Light).
 *
 * MODE 03 OVERVIEW:
 * - Returns confirmed/matured DTCs only (not pending codes)
 * - DTCs are emissions-related powertrain codes ("P0" and "P2" codes)
 * - First byte indicates number of DTCs stored
 * - Each DTC is 2 bytes in standardized format
 *
 * DTC FORMAT:
 * Byte 1 (High byte):
 *   Bits 7-6: DTC type (00=P0, 01=P2, 10=P3, 11=U codes)
 *   Bits 5-4: First digit of code
 *   Bits 3-0: Second digit of code
 * Byte 2 (Low byte):
 *   Bits 7-4: Third digit of code
 *   Bits 3-0: Fourth digit of code
 *
 * EXAMPLE:
 *   P0100 = 0x01 0x00 (MAF Circuit Malfunction)
 *   P0200 = 0x02 0x00 (Injector Circuit Malfunction)
 *
 * This is an emissions monitoring function required by EPA/CARB for OBD-II compliance.
 */

#include "../mode_registry.h"
#include <FlexCAN_T4.h>

// External CAN bus instance
extern FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

// External ECU data structures
extern ecu_t ecu;
extern freeze_frame_t freeze_frame[2];

/*
 * Mode 03 Handler - Request Emissions-Related Trouble Codes
 *
 * Returns stored DTCs that have triggered the MIL (Check Engine Light).
 * These are confirmed emissions faults requiring attention.
 *
 * Response format:
 * - No DTCs: buf[0]=0x02, buf[1]=0x43, buf[2]=0x00
 * - With DTCs: buf[0]=length, buf[1]=0x43, buf[2]=count, buf[3-6]=DTC bytes
 */
bool handle_mode_03(CAN_message_t& can_MsgRx, CAN_message_t& can_MsgTx, ecu_simClass* ecu_sim) {
    // Check if this is a Mode 03 request
    if (can_MsgRx.buf[1] != MODE3) {
        return false;  // Not our mode, let other handlers try
    }

    // Handle Mode 03 - Request trouble codes
    if (ecu.dtc == false) {
        // No DTCs stored - return empty response
        can_MsgTx.buf[0] = 0x02;              // Length: 2 bytes
        can_MsgTx.buf[1] = MODE3_RESPONSE;    // Mode 03 response (0x43)
        can_MsgTx.buf[2] = 0x00;              // 0 DTCs stored
    } else {
        // DTCs stored - return the trouble codes
        can_MsgTx.buf[0] = 0x06;              // Length: 6 bytes
        can_MsgTx.buf[1] = MODE3_RESPONSE;    // Mode 03 response (0x43)
        can_MsgTx.buf[2] = 0x02;              // Number of DTCs: 2

        // First DTC: P0100 (MAF Circuit Malfunction)
        // 0x0100 = P0100 in OBD-II DTC format
        can_MsgTx.buf[3] = 0x01;              // High byte: 0001 0000 = P01xx
        can_MsgTx.buf[4] = 0x00;              // Low byte: 0000 0000 = P0100

        // Second DTC: P0200 (Injector Circuit Malfunction)
        // 0x0200 = P0200 in OBD-II DTC format
        can_MsgTx.buf[5] = 0x02;              // High byte: 0000 0010 = P02xx
        can_MsgTx.buf[6] = 0x00;              // Low byte: 0000 0000 = P0200
    }

    // Send response on standard OBD-II reply channel
    can_MsgTx.id = PID_REPLY;  // 0x7E8 - Engine ECU response
    can_MsgTx.len = 8;
    can1.write(can_MsgTx);

    return true;  // Mode 03 request handled successfully
}

// Register Mode 03 handler with the mode registry
// This enables automatic dispatch when Mode 03 requests are received
static ModeRegistrar mode_03_registrar(MODE3, handle_mode_03, "Request Emissions DTCs");
