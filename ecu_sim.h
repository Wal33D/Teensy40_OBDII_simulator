#ifndef ecu_sim__h
#define ecu_sim__h

#include <Arduino.h>

/* OBD-II PID definitions */
#define MODE1               0x01  // Show current data
#define MODE2               0x02  // Show freeze frame data
#define MODE3               0x03  // Show stored Diagnostic Trouble Codes
#define MODE4               0x04  // Clear Diagnostic Trouble Codes
#define MODE9               0x09  // Request vehicle information

#define PID_SUPPORTED       0x00
#define MONITOR_STATUS      0x01
#define ENGINE_LOAD         0x04  // <--- NEW: Engine load PID
#define ENGINE_COOLANT_TEMP 0x05
#define ENGINE_RPM          0x0C
#define VEHICLE_SPEED       0x0D
#define MAF_SENSOR          0x10
#define THROTTLE            0x11
#define O2_VOLTAGE          0x14
#define INTAKE_AIR_TEMP     0x0F  // <--- NEW: Intake air temp PID

// Mode 9 sub-PIDs
#define VIN_PID                 0x02
#define CALIBRATION_ID          0x04
#define CALIBRATION_VERIFICATION 0x06

#define MODE1_RESPONSE      0x41
#define MODE3_RESPONSE      0x43
#define MODE4_RESPONSE      0x44

#define PID_REQUEST         0x7DF
#define PID_REPLY           0x7E8

// ECU Physical ID used for flow control responses (typically 0x7E0)
#define ECU_PHYS_ID         0x7E0

// Pin definitions
static const int LED_red   = 9;
static const int LED_green = 8;
static const int SW1       = 6;
static const int SW2       = 7;

// Analog input definitions
static const int AN1 = 0;
static const int AN2 = 1;
static const int AN3 = 2;
static const int AN4 = 3;
static const int AN5 = 6;
static const int AN6 = 7;
static const int AN7 = 8;  // Extra analog channel (e.g., for engine load/intake air temp)

/**
 * Data structure representing various simulated PID values.
 */
typedef struct {
  unsigned char coolant_temp;      // 1 byte (0–255, offset -40 in real OBD)
  unsigned int  engine_rpm;        // 2 bytes
  unsigned char throttle_position; // 1 byte
  unsigned char vehicle_speed;     // 1 byte
  unsigned int  maf_airflow;       // 2 bytes
  unsigned int  o2_voltage;        // 2 bytes
  unsigned char dtc;               // 0 = no DTC, 1 = DTC present

  // NEW FIELDS
  unsigned char engine_load;       // 1 byte, typical 0–255 => 0–100% load
  unsigned char intake_air_temp;   // 1 byte, typical 0–255 => (A - 40) °C
} ecu_t;

/**
 * Extern the global 'ecu' variable so it's shared among files.
 */
extern ecu_t ecu;

/**
 * Global variables for VIN simulation.
 * - simulated_vin is mutable and updated (randomized) when SW2 is pressed.
 * - simulated_calid and simulated_cvn remain constant.
 */
extern char simulated_vin[18];
extern const char simulated_calid[];
extern const char simulated_cvn[];

/**
 * Main ECU simulator class.
 *
 * The ECU simulator handles sensor updates (update_pots) and processes incoming CAN messages (update).
 * In the updated functionality, pressing SW2 will randomize:
 *  - The VIN (selected from a hardcoded array)
 *  - The engine load and intake air temperature values.
 */
class ecu_simClass {
  public:
    ecu_simClass();
    uint8_t init(uint32_t baud);
    uint8_t update(void);
    void update_pots(void);
};

extern ecu_simClass ecu_sim;

#endif
