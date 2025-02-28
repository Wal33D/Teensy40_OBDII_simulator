#ifndef ECU_SIM_H
#define ECU_SIM_H

#include <Arduino.h>
#include <FlexCAN_T4.h>

// --- Pin Definitions ---
#define SW1           6
#define SW2           7
#define LED_red       9
#define LED_green     8

// --- Analog Channel Definitions ---
#define AN1           A1
#define AN2           A2
#define AN3           A3
#define AN4           A4
#define AN5           A5
#define AN6           A6
#define AN7           A7

// --- OBD-II and Protocol Definitions ---
#define PID_REQUEST           0x7DF
#define PID_REPLY             0x7E8
#define ECU_PHYS_ID           0x7E0

#define MODE1                 0x01
#define MODE1_RESPONSE        0x41

#define MODE3                 0x03
#define MODE3_RESPONSE        0x43

#define MODE4                 0x04
#define MODE4_RESPONSE        0x44

#define MODE9                 0x09
#define VIN_PID               0x02
#define CALIBRATION_ID        0x0A
#define CALIBRATION_VERIFICATION 0x0B

#define PID_SUPPORTED         0x00
#define MONITOR_STATUS        0x01
#define ENGINE_RPM            0x0C
#define ENGINE_COOLANT_TEMP   0x05
#define VEHICLE_SPEED         0x0D
#define MAF_SENSOR            0x10
#define O2_VOLTAGE            0x14
#define THROTTLE              0x11
// For demonstration purposes:
#define ENGINE_LOAD           0x04
#define INTAKE_AIR_TEMP       0x0F

// --- ECU Data Structure ---
typedef struct {
  uint16_t engine_rpm;
  uint8_t vehicle_speed;
  uint8_t coolant_temp;
  uint16_t maf_airflow;
  uint8_t throttle_position;
  uint16_t o2_voltage;
  uint8_t engine_load;
  uint8_t intake_air_temp;
  uint8_t dtc;
} ecu_t;

// --- Declare Global Variables for VIN Simulation ---
// These are referenced in the CAN helper module.
extern char simulated_vin[18];
extern const char simulated_calid[];
extern const char simulated_cvn[];

// --- ECU Simulator Class ---
class ecu_simClass {
public:
  ecu_t ecu;
  ecu_simClass();
  uint8_t init(uint32_t baud);
  void update_pots(void);
  uint8_t update(void);
};

extern ecu_simClass ecu_sim;

#endif // ECU_SIM_H
