#ifndef ecu_sim__h
#define ecu_sim__h

#include <Arduino.h>

/*
 * OBD-II ECU Simulator - Emissions Program Implementation
 *
 * IMPORTANT: OBD-II is designed as an emissions monitoring program, not a general
 * diagnostic system. It only covers emissions-related functions (engine, transmission,
 * drivetrain). Body controls, ABS, airbags, and lighting are manufacturer-specific.
 *
 * This simulator implements emissions-related OBD-II modes as defined by SAE J1979
 * and adopted by EPA/CARB for implementation since January 1, 1996.
 *
 * Details from http://en.wikipedia.org/wiki/OBD-II_PIDs
 */

/*
 * OBD-II MODES IMPLEMENTATION STATUS:
 *
 * Mode 01 (0x01) - Request Current Powertrain Data
 *   STATUS: FULLY IMPLEMENTED (44 PIDs)
 *   PURPOSE: Access to current LIVE emissions-related data values.
 *   NOTE: Must be actual readings, not default/substitute values.
 *
 * Mode 02 (0x02) - Request Freeze Frame Data
 *   STATUS: IMPLEMENTED
 *   PURPOSE: Access emissions data stored when related DTC was set.
 *   NOTE: Captures snapshot of conditions when fault occurred.
 *
 * Mode 03 (0x03) - Request Emissions-Related DTCs
 *   STATUS: IMPLEMENTED
 *   PURPOSE: Access stored "P" codes that triggered MIL.
 *   NOTE: Only "matured" DTCs as defined by OBD-II standards.
 *
 * Mode 04 (0x04) - Clear/Reset Emissions Diagnostic Info
 *   STATUS: IMPLEMENTED
 *   PURPOSE: Clear DTCs, freeze frames, test data, reset monitors, turn off MIL.
 *   NOTE: Comprehensive reset of all emissions diagnostic data.
 *
 * Mode 05 (0x05) - Oxygen Sensor Monitoring Test Results
 *   STATUS: NOT IMPLEMENTED (Legacy mode)
 *   PURPOSE: O2 sensor test results (replaced by Mode 06 on CAN systems).
 *   NOTE: Not available on CAN-based vehicles (2008+).
 *
 * Mode 06 (0x06) - On-Board Monitoring Test Results
 *   STATUS: NOT IMPLEMENTED
 *   PURPOSE: Test results for continuously/non-continuously monitored systems.
 *   NOTE: Data format is manufacturer-specific.
 *
 * Mode 07 (0x07) - Pending DTCs (Current Drive Cycle)
 *   STATUS: NOT IMPLEMENTED
 *   PURPOSE: Access codes from first drive cycle after ECM reset.
 *   NOTE: Shows "pending" codes before they mature.
 *
 * Mode 08 (0x08) - Bidirectional Control
 *   STATUS: NOT IMPLEMENTED
 *   PURPOSE: Control on-board systems for testing (mainly EVAP).
 *   NOTE: Limited implementation in real vehicles.
 *
 * Mode 09 (0x09) - Request Vehicle Information
 *   STATUS: FULLY IMPLEMENTED
 *   PURPOSE: Access VIN and calibration IDs from emissions modules.
 *   NOTE: Includes multi-frame ISO-TP protocol for long messages.
 *
 * Mode 10 (0x0A) - Permanent DTCs
 *   STATUS: NOT IMPLEMENTED
 *   PURPOSE: DTCs that only module can clear after successful test.
 *   NOTE: Remains even after Mode 04 clear until self-test passes.
 */

#define MODE1               0x01        // Current powertrain data (live emissions data)
#define MODE2               0x02        // Freeze frame (emissions data when DTC set)
#define MODE3               0x03        // Emissions-related DTCs ("P" codes)
#define MODE4               0x04        // Clear emissions diagnostic information
#define MODE9               0x09        // Vehicle information (VIN, calibrations)

/*
 * MODE 01 PID DEFINITIONS - Live Emissions Data
 * These PIDs provide real-time emissions-related engine data
 */
#define PID_SUPPORTED       0x00        // Bit-encoded PIDs supported [01-20]
#define MONITOR_STATUS      0x01        // Emissions monitor status since DTCs cleared
#define FUEL_SYSTEM_STATUS  0x03        // Fuel system status (open/closed loop)
#define CALCULATED_LOAD     0x04        // Engine load value for emissions calculations
#define ENGINE_COOLANT_TEMP 0x05        // Coolant temp affects emissions control
#define SHORT_FUEL_TRIM_1   0x06
#define LONG_FUEL_TRIM_1    0x07
#define SHORT_FUEL_TRIM_2   0x08
#define LONG_FUEL_TRIM_2    0x09
#define INTAKE_PRESSURE     0x0B
#define ENGINE_RPM          0x0C
#define VEHICLE_SPEED       0x0D
#define TIMING_ADVANCE      0x0E
#define INTAKE_AIR_TEMP     0x0F
#define MAF_SENSOR          0x10
#define THROTTLE            0x11
#define O2_SENSORS_PRESENT  0x13
#define O2_VOLTAGE          0x14
#define O2_SENSOR_2_B1      0x15
#define O2_SENSOR_2_B2      0x19
#define OBD_STANDARD        0x1C
#define ENGINE_RUN_TIME     0x1F

/*
 * Extended Mode 01 PIDs (0x20-0x60)
 * Additional emissions monitoring parameters
 */
#define PID_20_SUPPORTED    0x20        // Bit-encoded PIDs supported [21-40]
#define DISTANCE_WITH_MIL   0x21
#define FUEL_RAIL_PRESSURE  0x23
#define EVAP_PURGE         0x2E
#define FUEL_LEVEL         0x2F
#define WARM_UPS           0x30
#define DISTANCE_SINCE_CLR  0x31
#define EVAP_VAPOR_PRESS   0x32
#define BAROMETRIC_PRESS   0x33
#define O2_SENSOR_1_B1     0x34
#define O2_SENSOR_5_B2     0x38
#define CAT_TEMP_B1S1      0x3C
#define CAT_TEMP_B2S1      0x3D
#define PID_40_SUPPORTED   0x40
#define MONITOR_STATUS_CYC 0x41
#define CONTROL_MOD_VOLT   0x42
#define ABSOLUTE_LOAD      0x43
#define COMMANDED_EQUIV    0x44
#define REL_THROTTLE_POS   0x45
#define AMBIENT_AIR_TEMP   0x46
#define THROTTLE_POS_B     0x47
#define ACCEL_POS_D        0x49
#define ACCEL_POS_E        0x4A
#define COMMANDED_THROTTLE 0x4C
#define FUEL_TYPE          0x51
#define SHORT_O2_TRIM_B1   0x56
#define SHORT_O2_TRIM_B2   0x58

#define MODE1_RESPONSE      0x41
#define MODE2_RESPONSE      0x42
#define MODE3_RESPONSE      0x43
#define MODE4_RESPONSE      0x44
#define MODE9_RESPONSE      0x49

/*
 * MODE 09 PIDs - Vehicle Information
 * Required for emissions compliance tracking and verification
 */
#define VEH_INFO_SUPPORTED  0x00        // Supported Mode 09 PIDs
#define VIN_REQUEST         0x02        // Vehicle Identification Number (17 chars)
#define CAL_ID_REQUEST      0x04        // Calibration ID for emissions software
#define CVN_REQUEST         0x06        // Calibration Verification Number (checksum)
#define PERF_TRACK_REQUEST  0x08        // In-use performance tracking for monitors
#define ECU_NAME_REQUEST    0x0A        // ECU name/identifier
#define AUX_IO_REQUEST      0x14        // Auxiliary input/output status

/*
 * CAN ID Definitions for OBD-II Protocol
 * OBD-II uses specific CAN IDs for diagnostic communication:
 * - 0x7DF: Functional (broadcast) request to all ECUs
 * - 0x7E0-0x7E7: Physical request to specific ECUs
 * - 0x7E8-0x7EF: Response from ECUs (8 possible modules)
 */
#define PID_REQUEST         0x7DF       // Functional broadcast request
#define PID_REPLY_ENGINE    0x7E8       // Engine/Powertrain ECU response
#define PID_REPLY_TRANS     0x7E9       // Transmission ECU response
#define PID_REPLY_HYBRID    0x7EA       // Hybrid/Electric ECU response
#define PID_REPLY_CHASSIS   0x7EB       // Chassis/Body ECU response
#define PID_REPLY           PID_REPLY_ENGINE  // Default reply ID (engine)

// ISO-TP (ISO 15765-2) Protocol Control Information
#define ISO_TP_SINGLE_FRAME    0x00     // Single frame (0-7 data bytes)
#define ISO_TP_FIRST_FRAME     0x10     // First frame of multi-frame
#define ISO_TP_CONSEC_FRAME    0x20     // Consecutive frame
#define ISO_TP_FLOW_CONTROL    0x30     // Flow control frame

// Flow Control parameters
#define FC_CONTINUE         0x00        // Continue to send
#define FC_WAIT             0x01        // Wait for next flow control
#define FC_OVERFLOW         0x02        // Buffer overflow
// ISO-TP timing parameters (milliseconds)
#define ISO_TP_STMIN        10          // Minimum separation time between frames
#define ISO_TP_BS           0           // Block size (0 = send all frames)

static const int LED_red = 9;
static const int LED_green = 8;


static const int SW1 = 6;
static const int SW2 = 7;

static const int AN1 = 0;
static const int AN2 = 1;
static const int AN3 = 2;
static const int AN4 = 3;
static const int AN5 = 6;
static const int AN6 = 7;

/*
 * ISO-TP Transfer State Machine
 * Manages multi-frame message transfers per ISO 15765-2
 */
typedef enum {
    ISOTP_IDLE,           // No transfer in progress
    ISOTP_WAIT_FC,        // Waiting for flow control after sending first frame
    ISOTP_SENDING_CF,     // Sending consecutive frames
    ISOTP_WAIT_NEXT_FC,   // Waiting for next flow control (block size reached)
    ISOTP_ERROR           // Transfer error/abort
} isotp_state_t;

/*
 * ISO-TP Transfer Context
 * Maintains state for ongoing multi-frame transfers
 */
typedef struct {
    isotp_state_t state;          // Current transfer state
    uint8_t data[256];            // Buffer for complete message
    uint16_t total_len;           // Total message length
    uint16_t offset;              // Current position in buffer
    uint8_t seq_num;              // Next consecutive frame sequence number
    uint8_t block_size;           // Frames to send before next FC
    uint8_t blocks_sent;          // Frames sent in current block
    uint8_t st_min;               // Minimum separation time (ms)
    uint32_t last_frame_time;     // Timestamp of last frame sent
    uint32_t fc_wait_start;       // When we started waiting for FC
    uint16_t response_id;         // CAN ID to use for responses
    uint8_t mode;                 // OBD mode being serviced
    uint8_t pid;                  // PID being serviced
} isotp_transfer_t;

/*
 * ECU Data Structure
 * Holds current emissions-related sensor values
 */
typedef struct{
        unsigned char coolant_temp;      // Affects cold-start emissions
        unsigned int engine_rpm;          // Engine speed for load calculations
        unsigned char throttle_position;  // Throttle affects fuel mixture
        unsigned char vehicle_speed;      // Speed for emissions testing modes
        unsigned int maf_airflow;         // Mass Air Flow for fuel calculations
        unsigned int o2_voltage;          // O2 sensor for emissions feedback
        unsigned char dtc;                // Diagnostic trouble code storage
}ecu_t;

/*
 * Freeze Frame Structure (Mode 02)
 * Captures emissions data snapshot when DTC is triggered
 * Required by OBD-II to help diagnose intermittent faults
 */
typedef struct{
        unsigned char coolant_temp;      // Engine temp when fault occurred
        unsigned int engine_rpm;          // RPM at time of fault
        unsigned char throttle_position;  // Throttle position during fault
        unsigned char vehicle_speed;      // Speed when emissions fault detected
        unsigned int maf_airflow;         // Airflow reading at fault time
        unsigned int o2_voltage;          // O2 voltage when DTC set
        bool data_stored;                 // Flag: freeze frame contains valid data
        unsigned int dtc_code;            // Emissions DTC that triggered capture
}freeze_frame_t;

extern ecu_t ecu;
extern freeze_frame_t freeze_frame[2];  // Support 2 freeze frames
extern isotp_transfer_t isotp_tx;        // ISO-TP transmit context

class ecu_simClass
{
  
  public:

  ecu_simClass();
  uint8_t init(uint32_t baud);
  uint8_t update(void);
  void update_pots(void);
  void isotp_init_transfer(uint8_t* data, uint16_t len, uint16_t can_id, uint8_t mode, uint8_t pid);
  void isotp_send_first_frame(void);
  void isotp_handle_flow_control(uint8_t* data);
  void isotp_send_consecutive_frame(void);
  void isotp_process_transfers(void);

private:
  
};

extern ecu_simClass ecu_sim;
#endif
