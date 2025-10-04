/*
 * OBD-II Mode 09 - Request Vehicle Information
 *
 * EMISSIONS COMPLIANCE CONTEXT:
 * ============================
 * Mode 09 is a critical component of the OBD-II emissions monitoring program.
 * It provides vehicle and calibration identification data required for:
 *
 * 1. EMISSIONS CERTIFICATION & COMPLIANCE VERIFICATION
 *    - VIN (0x02): Links the vehicle to EPA/CARB certification documents
 *    - Ensures the correct emissions standards are being applied
 *    - Required for emissions testing and inspection programs
 *
 * 2. EMISSIONS SOFTWARE VALIDATION
 *    - Calibration ID (0x04): Identifies the emissions control software version
 *    - CVN (0x06): Verifies software hasn't been modified/tampered with
 *    - Prevents unauthorized modifications that could increase emissions
 *    - Critical for detecting "defeat devices" or illegal tuning
 *
 * 3. EMISSIONS MONITOR PERFORMANCE TRACKING (0x08)
 *    - Tracks how often emissions monitors run vs. opportunities
 *    - Ensures monitors are functioning and detecting faults
 *    - Required by EPA to verify OBD-II system effectiveness
 *    - Helps identify systems that might not be monitoring properly
 *
 * 4. MULTI-ECU EMISSIONS SYSTEM IDENTIFICATION
 *    - ECU Name (0x0A): Identifies which module controls what emissions function
 *    - Required because modern vehicles have multiple ECUs handling emissions:
 *      * Engine ECU - primary emissions control
 *      * Transmission ECU - affects emissions through shift patterns
 *      * Hybrid ECU - emissions during electric/gas transitions
 *
 * REGULATORY IMPORTANCE:
 * All Mode 09 data is legally required under EPA/CARB regulations. This data
 * enables:
 * - State emissions inspection programs (I/M programs)
 * - EPA compliance auditing and enforcement
 * - Detection of illegal emissions control modifications
 * - Verification that vehicles meet their certified emissions standards
 *
 * ISO-TP MULTI-FRAME PROTOCOL:
 * Many Mode 09 responses exceed 8 bytes (CAN frame limit), requiring ISO-TP
 * (ISO 15765-2) multi-frame protocol. This implementation properly handles:
 * - First Frame transmission
 * - Flow Control reception from tester
 * - Consecutive Frame transmission with proper timing
 * - Multiple ECU responses (engine, transmission, hybrid)
 */

#include "../mode_registry.h"
#include <FlexCAN_T4.h>

// External CAN bus instance
extern FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

// External ECU data structures (required for Mode 09)
extern ecu_t ecu;
extern freeze_frame_t freeze_frame[2];
extern isotp_transfer_t isotp_tx;

/*
 * Mode 09 Handler - Vehicle Information
 *
 * Handles all Mode 09 PID requests for vehicle identification and
 * emissions calibration data. Implements ISO-TP multi-frame protocol
 * for responses that exceed single CAN frame (8 bytes).
 *
 * MULTI-ECU BEHAVIOR:
 * Some PIDs (like 0x00 - supported PIDs) trigger responses from multiple
 * ECUs to allow scan tools to discover all emissions control modules.
 * Other PIDs are responded to by the specific ECU that owns that data.
 */
bool handle_mode_09(CAN_message_t& can_MsgRx, CAN_message_t& can_MsgTx, ecu_simClass* ecu_sim) {
    // Check if this is a Mode 09 request
    if (can_MsgRx.buf[1] != MODE9) {
        return false;  // Not our mode, let other handlers try
    }

    // Default to Engine ECU response (most Mode 09 data comes from engine ECU)
    can_MsgTx.id = PID_REPLY_ENGINE;
    can_MsgTx.len = 8;
    can_MsgTx.buf[1] = MODE9_RESPONSE;

    // Check for ISO-TP flow control frames
    // These are sent by the tester to control multi-frame message flow
    if((can_MsgRx.buf[0] & 0xF0) == ISO_TP_FLOW_CONTROL) {
        // Handle flow control - continue sending consecutive frames
        // This would be implemented based on pending multi-frame transfers
        // For now, we'll handle it in the multi-frame sections
        return false;  // Let main update loop handle via isotp_handle_flow_control
    }

    switch(can_MsgRx.buf[2])  // PID is in buf[2]
    {
        case VEH_INFO_SUPPORTED:  // 0x00 - Supported PIDs
            /*
             * MULTI-ECU RESPONSE FOR EMISSIONS SYSTEM DISCOVERY
             * Multiple ECUs respond to show which emissions modules are present.
             * This allows scan tools to identify all emissions control modules
             * and query each one for their specific vehicle information.
             */

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
            /*
             * EMISSIONS COMPLIANCE: VIN
             * The VIN is essential for emissions compliance verification.
             * It links this vehicle to its EPA/CARB certification documents,
             * which specify the emissions standards the vehicle must meet.
             * State inspection programs use VIN to verify proper emissions equipment.
             *
             * FORMAT: 17 characters (requires multi-frame ISO-TP)
             * Total message: 3 header bytes + 17 VIN bytes = 20 bytes
             */
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

                // Initialize ISO-TP transfer for multi-frame response
                ecu_sim->isotp_init_transfer(vin_data, 20, PID_REPLY_ENGINE, MODE9, VIN_REQUEST);
                ecu_sim->isotp_send_first_frame();
            }
            break;

        case CAL_ID_REQUEST:  // 0x04 - Calibration ID
            /*
             * EMISSIONS COMPLIANCE: CALIBRATION ID
             * Identifies the specific version of emissions control software.
             * Critical for:
             * - Verifying vehicle has correct emissions calibration
             * - Detecting unauthorized software modifications
             * - Ensuring recalls/updates are properly applied
             * - EPA enforcement of emissions standards
             *
             * FORMAT: 16 characters (requires multi-frame ISO-TP)
             * Total message: 3 header bytes + 16 cal ID bytes = 19 bytes
             */
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

                // Initialize ISO-TP transfer for multi-frame response
                ecu_sim->isotp_init_transfer(cal_data, 19, PID_REPLY_ENGINE, MODE9, CAL_ID_REQUEST);
                ecu_sim->isotp_send_first_frame();
            }
            break;

        case CVN_REQUEST:  // 0x06 - Calibration Verification Number
            /*
             * EMISSIONS COMPLIANCE: CALIBRATION VERIFICATION NUMBER (CVN)
             * A cryptographic checksum of the emissions calibration software.
             * Absolutely critical for detecting "defeat devices" and illegal tuning.
             *
             * The CVN allows EPA and state inspectors to verify that:
             * - Emissions software hasn't been tampered with
             * - No unauthorized modifications have been made
             * - The calibration matches the certified configuration
             *
             * This was central to detecting the VW "Dieselgate" scandal.
             *
             * FORMAT: 4 bytes (fits in single frame)
             */
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
            /*
             * EMISSIONS COMPLIANCE: ECU IDENTIFICATION
             * Identifies which ECU/module controls what emissions function.
             * Required because modern vehicles have multiple modules involved
             * in emissions control:
             * - Engine ECU: Primary emissions control (fuel, ignition, etc.)
             * - Transmission ECU: Shift patterns affect emissions
             * - Hybrid ECU: Manages electric/gas transitions
             *
             * This helps technicians and inspectors understand which module
             * is responsible for specific emissions monitoring functions.
             *
             * FORMAT: Variable length string (requires multi-frame ISO-TP)
             * Total message: 23 bytes for "ECM-EngineControl"
             */
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

                // Initialize ISO-TP transfer for multi-frame response
                ecu_sim->isotp_init_transfer(name_data, 23, PID_REPLY_ENGINE, MODE9, ECU_NAME_REQUEST);
                ecu_sim->isotp_send_first_frame();
            }
            break;

        case PERF_TRACK_REQUEST:  // 0x08 - Performance Tracking
            /*
             * EMISSIONS COMPLIANCE: IN-USE PERFORMANCE TRACKING
             * Tracks how often emissions monitors run vs. how often they could run.
             * This is REQUIRED by EPA to ensure the OBD-II system is actually
             * monitoring for emissions faults, not just pretending to.
             *
             * For each monitor (catalyst, O2 sensor, EVAP, etc.), tracks:
             * - Numerator: How many times the monitor completed
             * - Denominator: How many times it had the opportunity to run
             *
             * If ratios are too low, it indicates the OBD system isn't working
             * properly and the vehicle may be emitting excess pollutants without
             * detection. EPA uses this data to identify problematic designs.
             *
             * This prevents manufacturers from creating monitors that rarely run,
             * thus avoiding detection of emissions faults.
             *
             * FORMAT: 43 bytes of performance tracking data (requires multi-frame)
             */
            {
                // Only start transfer if not already in progress
                if(isotp_tx.state != ISOTP_IDLE) break;

                // Performance tracking data: 43 bytes total
                uint8_t perf_data[43];
                perf_data[0] = MODE9_RESPONSE;
                perf_data[1] = PERF_TRACK_REQUEST;

                // Real performance tracking data from Mercedes-Benz
                // Format: pairs of (numerator, denominator) for each monitor
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

                // Initialize ISO-TP transfer for multi-frame response
                ecu_sim->isotp_init_transfer(perf_data, 43, PID_REPLY_ENGINE, MODE9, PERF_TRACK_REQUEST);
                ecu_sim->isotp_send_first_frame();
            }
            break;

        case AUX_IO_REQUEST:  // 0x14 - Auxiliary I/O Status
            /*
             * EMISSIONS COMPLIANCE: AUXILIARY INPUT/OUTPUT STATUS
             * Reports status of auxiliary inputs/outputs related to emissions.
             * This can include:
             * - PTO (Power Take-Off) status - affects emissions during operation
             * - Auxiliary emission control devices
             * - Special emissions control modes
             *
             * FORMAT: 5 bytes (fits in single frame)
             */
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

        default:
            // Unsupported Mode 09 PID - don't respond
            // (Scan tool will timeout if PID not supported)
            break;
    }

    return true;  // Mode 09 handled the request
}

// Register Mode 09 handler at compile time
// This makes the mode automatically available to the ECU simulator
static ModeRegistrar mode_09_registrar(MODE9, handle_mode_09, "Vehicle Information");
