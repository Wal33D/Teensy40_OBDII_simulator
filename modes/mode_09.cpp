/*
 * OBD-II Mode 09 - Request Vehicle Information
 *
 * MULTI-ECU SIMULATION: Mercedes-Benz GLE-Class Configuration
 * ============================================================
 * This implementation simulates 3 ECUs as found in a 2018 Mercedes-Benz GLE-Class:
 *
 * 1. ECM - Engine Control Module (0x7E8)
 *    - Calibration ID: 2769011200190170
 *    - Name: ECM-EngineControl
 *    - Function: Primary emissions control (fuel, ignition, catalytic converter)
 *    - VIN: 4JGDA5HB7JB158144
 *
 * 2. TCM - Transmission Control Module (0x7E9)
 *    - Calibration ID: 00090237271900001
 *    - Name: TCM-TransmisCtrl
 *    - Function: Transmission shift patterns affecting emissions
 *    - VIN: 4JGDA5HB7JB158144 (same as vehicle)
 *
 * 3. FPCM - Fuel Pump Control Module (0x7EB)
 *    - Calibration ID: 00090121001900560
 *    - Name: FPCM-FuelPumpCtrl
 *    - Function: Fuel delivery control for emissions optimization
 *    - VIN: 4JGDA5HB7JB158144 (same as vehicle)
 *
 * CAN ID MAPPING:
 * - Request IDs:  0x7DF (broadcast), 0x7E0 (ECM), 0x7E1 (TCM), 0x7E3 (FPCM)
 * - Response IDs: 0x7E8 (ECM), 0x7E9 (TCM), 0x7EB (FPCM)
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
             *
             * Simulating Mercedes-Benz GLE-Class with 3 ECUs:
             * - ECM-EngineControl (0x7E8)
             * - TCM-TransmisCtrl (0x7E9)
             * - FPCM-FuelPumpCtrl (0x7EB)
             */

            // ECM - Engine Control Module response (0x7E8)
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

            // TCM - Transmission Control Module response (0x7E9)
            can_MsgTx.id = PID_REPLY_TRANS;
            can_MsgTx.buf[0] = 0x06;  // Single frame, 6 bytes
            can_MsgTx.buf[2] = VEH_INFO_SUPPORTED;
            can_MsgTx.buf[3] = 0x54;  // Supports: 0x02, 0x04, 0x0A
            can_MsgTx.buf[4] = 0x40;
            can_MsgTx.buf[5] = 0x00;
            can_MsgTx.buf[6] = 0x00;
            can_MsgTx.buf[7] = 0x00;  // Padding
            can1.write(can_MsgTx);

            delay(5);

            // FPCM - Fuel Pump Control Module response (0x7EB)
            can_MsgTx.id = PID_REPLY_CHASSIS;
            can_MsgTx.buf[0] = 0x06;  // Single frame, 6 bytes
            can_MsgTx.buf[2] = VEH_INFO_SUPPORTED;
            can_MsgTx.buf[3] = 0x54;  // Supports: 0x02, 0x04, 0x0A
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
             *
             * MULTI-ECU RESPONSE: All ECUs respond with the same VIN
             */
            {
                // Check which ECU should respond based on request ID
                uint16_t response_id = PID_REPLY_ENGINE;  // Default

                // Determine target ECU from request
                if (can_MsgRx.id == PID_REQUEST_TRANS) {
                    response_id = PID_REPLY_TRANS;
                } else if (can_MsgRx.id == 0x7E3) {  // Request to FPCM
                    response_id = PID_REPLY_CHASSIS;
                } else if (can_MsgRx.id == PID_REQUEST) {
                    // Broadcast request - all ECUs respond
                    // We'll handle this by sending multiple responses

                    // VIN: 4JGDA5HB7JB158144 (17 chars)
                    uint8_t vin_data[20];  // 3 header + 17 VIN
                    vin_data[0] = MODE9_RESPONSE;
                    vin_data[1] = VIN_REQUEST;
                    vin_data[2] = 0x01;  // 1 data item
                    const char* vin = "4JGDA5HB7JB158144";
                    for(int i = 0; i < 17; i++) {
                        vin_data[i+3] = vin[i];
                    }

                    // Only send first ECU if no transfer in progress
                    if(isotp_tx.state == ISOTP_IDLE) {
                        // ECM response
                        ecu_sim->isotp_init_transfer(vin_data, 20, PID_REPLY_ENGINE, MODE9, VIN_REQUEST);
                        ecu_sim->isotp_send_first_frame();
                    }
                    break;  // Additional ECUs will respond when transfer completes
                }

                // Single ECU targeted request
                if(isotp_tx.state != ISOTP_IDLE) break;

                uint8_t vin_data[20];  // 3 header + 17 VIN
                vin_data[0] = MODE9_RESPONSE;
                vin_data[1] = VIN_REQUEST;
                vin_data[2] = 0x01;  // 1 data item
                const char* vin = "4JGDA5HB7JB158144";
                for(int i = 0; i < 17; i++) {
                    vin_data[i+3] = vin[i];
                }

                // Initialize ISO-TP transfer for multi-frame response
                ecu_sim->isotp_init_transfer(vin_data, 20, response_id, MODE9, VIN_REQUEST);
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
             * FORMAT: 16-17 characters (requires multi-frame ISO-TP)
             * Total message: 3 header bytes + cal ID bytes
             *
             * MULTI-ECU RESPONSE: Each ECU has unique calibration ID
             * - ECM-EngineControl: 2769011200190170 (16 chars)
             * - TCM-TransmisCtrl: 00090237271900001 (17 chars)
             * - FPCM-FuelPumpCtrl: 00090121001900560 (17 chars)
             */
            {
                // Check which ECU should respond based on request ID
                uint16_t response_id = PID_REPLY_ENGINE;  // Default
                const char* cal_id = "2769011200190170";  // ECM default
                uint8_t cal_len = 16;

                // Determine target ECU from request
                if (can_MsgRx.id == PID_REQUEST_TRANS) {
                    response_id = PID_REPLY_TRANS;
                    cal_id = "00090237271900001";  // TCM calibration ID
                    cal_len = 17;
                } else if (can_MsgRx.id == 0x7E3) {  // Request to FPCM
                    response_id = PID_REPLY_CHASSIS;
                    cal_id = "00090121001900560";  // FPCM calibration ID
                    cal_len = 17;
                } else if (can_MsgRx.id == PID_REQUEST) {
                    // Broadcast request - ALL 3 ECUs respond with their calibration IDs
                    // Start ECM immediately, queue TCM and FPCM

                    if(isotp_tx.state != ISOTP_IDLE) break;  // Transfer in progress

                    // ECM calibration ID
                    uint8_t ecm_cal_data[19];
                    ecm_cal_data[0] = MODE9_RESPONSE;
                    ecm_cal_data[1] = CAL_ID_REQUEST;
                    ecm_cal_data[2] = 0x01;
                    const char* ecm_cal = "2769011200190170";
                    for(int i = 0; i < 16; i++) ecm_cal_data[i+3] = ecm_cal[i];

                    // TCM calibration ID
                    uint8_t tcm_cal_data[20];
                    tcm_cal_data[0] = MODE9_RESPONSE;
                    tcm_cal_data[1] = CAL_ID_REQUEST;
                    tcm_cal_data[2] = 0x01;
                    const char* tcm_cal = "00090237271900001";
                    for(int i = 0; i < 17; i++) tcm_cal_data[i+3] = tcm_cal[i];

                    // FPCM calibration ID
                    uint8_t fpcm_cal_data[20];
                    fpcm_cal_data[0] = MODE9_RESPONSE;
                    fpcm_cal_data[1] = CAL_ID_REQUEST;
                    fpcm_cal_data[2] = 0x01;
                    const char* fpcm_cal = "00090121001900560";
                    for(int i = 0; i < 17; i++) fpcm_cal_data[i+3] = fpcm_cal[i];

                    // Start ECM transfer immediately
                    ecu_sim->isotp_init_transfer(ecm_cal_data, 19, PID_REPLY_ENGINE, MODE9, CAL_ID_REQUEST);
                    ecu_sim->isotp_send_first_frame();

                    // Queue TCM and FPCM transfers
                    ecu_sim->isotp_queue_transfer(tcm_cal_data, 20, PID_REPLY_TRANS, MODE9, CAL_ID_REQUEST);
                    ecu_sim->isotp_queue_transfer(fpcm_cal_data, 20, PID_REPLY_CHASSIS, MODE9, CAL_ID_REQUEST);

                    break;
                }

                // Single ECU targeted request
                if(isotp_tx.state != ISOTP_IDLE) break;

                uint8_t cal_data[20];  // 3 header + max 17 cal ID
                cal_data[0] = MODE9_RESPONSE;
                cal_data[1] = CAL_ID_REQUEST;
                cal_data[2] = 0x01;  // 1 data item

                // Copy calibration ID
                for(int i = 0; i < cal_len; i++) {
                    cal_data[i+3] = cal_id[i];
                }

                // Initialize ISO-TP transfer for multi-frame response
                ecu_sim->isotp_init_transfer(cal_data, 3 + cal_len, response_id, MODE9, CAL_ID_REQUEST);
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
             *
             * MULTI-ECU RESPONSE: All 3 ECUs respond with their unique CVNs
             * - ECM: EB854939
             * - TCM: 5DEF71AD
             * - FPCM: 8CD7FF6C
             */

            // ECM - Engine Control Module CVN
            can_MsgTx.id = PID_REPLY_ENGINE;
            can_MsgTx.buf[0] = 0x06;  // Single frame, 6 bytes
            can_MsgTx.buf[2] = CVN_REQUEST;
            can_MsgTx.buf[3] = 0x01;  // 1 CVN
            can_MsgTx.buf[4] = 0xEB;
            can_MsgTx.buf[5] = 0x85;
            can_MsgTx.buf[6] = 0x49;
            can_MsgTx.buf[7] = 0x39;
            can1.write(can_MsgTx);

            delay(5);  // Realistic delay between ECU responses

            // TCM - Transmission Control Module CVN
            can_MsgTx.id = PID_REPLY_TRANS;
            can_MsgTx.buf[0] = 0x06;  // Single frame, 6 bytes
            can_MsgTx.buf[2] = CVN_REQUEST;
            can_MsgTx.buf[3] = 0x01;  // 1 CVN
            can_MsgTx.buf[4] = 0x5D;
            can_MsgTx.buf[5] = 0xEF;
            can_MsgTx.buf[6] = 0x71;
            can_MsgTx.buf[7] = 0xAD;
            can1.write(can_MsgTx);

            delay(5);  // Realistic delay between ECU responses

            // FPCM - Fuel Pump Control Module CVN
            can_MsgTx.id = PID_REPLY_CHASSIS;
            can_MsgTx.buf[0] = 0x06;  // Single frame, 6 bytes
            can_MsgTx.buf[2] = CVN_REQUEST;
            can_MsgTx.buf[3] = 0x01;  // 1 CVN
            can_MsgTx.buf[4] = 0x8C;
            can_MsgTx.buf[5] = 0xD7;
            can_MsgTx.buf[6] = 0xFF;
            can_MsgTx.buf[7] = 0x6C;
            can1.write(can_MsgTx);
            break;

        case ECU_NAME_REQUEST:  // 0x0A - ECU Name
            /*
             * EMISSIONS COMPLIANCE: ECU IDENTIFICATION
             * Identifies which ECU/module controls what emissions function.
             * Required because modern vehicles have multiple modules involved
             * in emissions control:
             * - ECM: Engine Control Module - Primary emissions control
             * - TCM: Transmission Control Module - Shift patterns affect emissions
             * - FPCM: Fuel Pump Control Module - Fuel delivery for emissions
             *
             * This helps technicians and inspectors understand which module
             * is responsible for specific emissions monitoring functions.
             *
             * FORMAT: Variable length string (requires multi-frame ISO-TP)
             * MULTI-ECU RESPONSE: Each ECU returns its unique name
             * - ECM-EngineControl (23 bytes)
             * - TCM-TransmisCtrl (22 bytes)
             * - FPCM-FuelPumpCtrl (23 bytes)
             */
            {
                // Check which ECU should respond based on request ID
                uint16_t response_id = PID_REPLY_ENGINE;  // Default
                const char* ecu_prefix = "ECM";
                const char* ecu_name = "EngineControl";
                uint8_t total_len = 23;

                // Determine target ECU from request
                if (can_MsgRx.id == PID_REQUEST_TRANS) {
                    response_id = PID_REPLY_TRANS;
                    ecu_prefix = "TCM";
                    ecu_name = "TransmisCtrl";
                    total_len = 22;  // Slightly different length
                } else if (can_MsgRx.id == 0x7E3) {  // Request to FPCM
                    response_id = PID_REPLY_CHASSIS;
                    ecu_prefix = "FPCM";
                    ecu_name = "FuelPumpCtrl";
                    total_len = 23;
                } else if (can_MsgRx.id == PID_REQUEST) {
                    // Broadcast request - ALL 3 ECUs respond with their names
                    // Start ECM immediately, queue TCM and FPCM

                    if(isotp_tx.state != ISOTP_IDLE) break;  // Transfer in progress

                    // ECM Name: ECM-EngineControl
                    uint8_t ecm_name_data[23];
                    ecm_name_data[0] = MODE9_RESPONSE;
                    ecm_name_data[1] = ECU_NAME_REQUEST;
                    ecm_name_data[2] = 0x01;
                    ecm_name_data[3] = 'E'; ecm_name_data[4] = 'C'; ecm_name_data[5] = 'M';
                    ecm_name_data[6] = 0x00; ecm_name_data[7] = '-';
                    const char* ecm_nm = "EngineControl";
                    for(int i = 0; i < 13; i++) ecm_name_data[i+8] = ecm_nm[i];
                    ecm_name_data[21] = 0x00; ecm_name_data[22] = 0x00;

                    // TCM Name: TCM-TransmisCtrl
                    uint8_t tcm_name_data[22];
                    tcm_name_data[0] = MODE9_RESPONSE;
                    tcm_name_data[1] = ECU_NAME_REQUEST;
                    tcm_name_data[2] = 0x01;
                    tcm_name_data[3] = 'T'; tcm_name_data[4] = 'C'; tcm_name_data[5] = 'M';
                    tcm_name_data[6] = 0x00; tcm_name_data[7] = '-';
                    const char* tcm_nm = "TransmisCtrl";
                    for(int i = 0; i < 13; i++) tcm_name_data[i+8] = tcm_nm[i];
                    tcm_name_data[21] = 0x00;

                    // FPCM Name: FPCM-FuelPumpCtrl
                    uint8_t fpcm_name_data[24];
                    fpcm_name_data[0] = MODE9_RESPONSE;
                    fpcm_name_data[1] = ECU_NAME_REQUEST;
                    fpcm_name_data[2] = 0x01;
                    fpcm_name_data[3] = 'F'; fpcm_name_data[4] = 'P'; fpcm_name_data[5] = 'C'; fpcm_name_data[6] = 'M';
                    fpcm_name_data[7] = 0x00; fpcm_name_data[8] = '-';
                    const char* fpcm_nm = "FuelPumpCtrl";
                    for(int i = 0; i < 12; i++) fpcm_name_data[i+9] = fpcm_nm[i];
                    fpcm_name_data[21] = 0x00; fpcm_name_data[22] = 0x00; fpcm_name_data[23] = 0x00;

                    // Start ECM transfer immediately
                    ecu_sim->isotp_init_transfer(ecm_name_data, 23, PID_REPLY_ENGINE, MODE9, ECU_NAME_REQUEST);
                    ecu_sim->isotp_send_first_frame();

                    // Queue TCM and FPCM transfers
                    ecu_sim->isotp_queue_transfer(tcm_name_data, 22, PID_REPLY_TRANS, MODE9, ECU_NAME_REQUEST);
                    ecu_sim->isotp_queue_transfer(fpcm_name_data, 24, PID_REPLY_CHASSIS, MODE9, ECU_NAME_REQUEST);

                    break;
                }

                // Single ECU targeted request
                if(isotp_tx.state != ISOTP_IDLE) break;

                // Build ECU Name message
                uint8_t name_data[25];  // Max size for any ECU name
                name_data[0] = MODE9_RESPONSE;
                name_data[1] = ECU_NAME_REQUEST;
                name_data[2] = 0x01;  // 1 data item

                // Copy ECU prefix (ECM, TCM, or FPCM)
                int idx = 3;
                for(int i = 0; ecu_prefix[i] != '\0'; i++) {
                    name_data[idx++] = ecu_prefix[i];
                }
                name_data[idx++] = 0x00;  // Separator
                name_data[idx++] = '-';

                // Copy ECU name
                for(int i = 0; ecu_name[i] != '\0'; i++) {
                    name_data[idx++] = ecu_name[i];
                }

                // Pad to expected length
                while(idx < total_len) {
                    name_data[idx++] = 0x00;
                }

                // Initialize ISO-TP transfer for multi-frame response
                ecu_sim->isotp_init_transfer(name_data, total_len, response_id, MODE9, ECU_NAME_REQUEST);
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
