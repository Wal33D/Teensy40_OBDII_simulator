#ifndef ecu_sim__h
#define ecu_sim__h

#include <Arduino.h>

 /* Details from http://en.wikipedia.org/wiki/OBD-II_PIDs */
#define MODE1               0x01        //Show current data
#define MODE2               0x02        //Show freeze frame data
#define MODE3               0x03        //Show stored Diagnostic Trouble Codes
#define MODE4               0x04        //Clear Diagnostic Trouble Codes and stored values
#define MODE9               0x09        //Request vehicle information

#define PID_SUPPORTED       0x00
#define MONITOR_STATUS      0x01
#define FUEL_SYSTEM_STATUS  0x03
#define CALCULATED_LOAD     0x04
#define ENGINE_COOLANT_TEMP 0x05
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

// Extended Mode 01 PIDs
#define PID_20_SUPPORTED    0x20
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

// Mode 09 PIDs
#define VEH_INFO_SUPPORTED  0x00
#define VIN_REQUEST         0x02
#define CAL_ID_REQUEST      0x04
#define CVN_REQUEST         0x06
#define PERF_TRACK_REQUEST  0x08
#define ECU_NAME_REQUEST    0x0A
#define AUX_IO_REQUEST      0x14
#define PID_REQUEST         0x7DF
#define PID_REPLY           0x7E8
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

typedef struct{
        unsigned char coolant_temp;
        unsigned int engine_rpm;
        unsigned char throttle_position;
        unsigned char vehicle_speed;
        unsigned int maf_airflow;
        unsigned int o2_voltage;
        unsigned char dtc;
}ecu_t;

typedef struct{
        unsigned char coolant_temp;
        unsigned int engine_rpm;
        unsigned char throttle_position;
        unsigned char vehicle_speed;
        unsigned int maf_airflow;
        unsigned int o2_voltage;
        bool data_stored;  // Flag to indicate if freeze frame data is stored
        unsigned int dtc_code;  // Which DTC triggered this freeze frame
}freeze_frame_t;

extern ecu_t ecu;
extern freeze_frame_t freeze_frame[2];  // Support 2 freeze frames

class ecu_simClass
{
  
  public:

  ecu_simClass();
  uint8_t init(uint32_t baud);
  uint8_t update(void);
  void update_pots(void);

private:
  
};

extern ecu_simClass ecu_sim;
#endif
