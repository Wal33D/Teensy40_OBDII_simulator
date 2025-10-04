# Mode 01 - Current Powertrain Diagnostic Data

## Overview

Mode 01 provides access to **current LIVE emissions-related data values** from the vehicle's powertrain control modules. This is the most frequently used OBD-II mode, as it allows real-time monitoring of all emissions-critical parameters during vehicle operation.

**Important**: All Mode 01 data must be actual sensor readings, not default or substitute values. The data reflects the instantaneous state of the emissions control systems as required by EPA/CARB regulations.

## Purpose in OBD-II Emissions Monitoring

Mode 01 serves several critical functions in the OBD-II emissions monitoring program:

1. **Real-time Emissions Diagnosis**: Allows technicians to observe current emissions system operation
2. **Closed-Loop Verification**: Confirms fuel trim and O2 sensor feedback are maintaining stoichiometric ratio (14.7:1 air-fuel ratio)
3. **Catalyst Efficiency**: Monitors temperatures and O2 sensor switching to verify catalyst operation
4. **EVAP System**: Tracks evaporative emissions control system operation
5. **Monitor Status**: Shows which emissions readiness monitors have completed their self-tests
6. **Drive Cycle Preparation**: Helps technicians prepare vehicles for I/M (Inspection and Maintenance) testing

## Supported PIDs (45 Total)

This implementation includes 45 PIDs based on real Mercedes-Benz vehicle data and SAE J1979 specifications.

### PID Support Indicators (PIDs 0x00, 0x20, 0x40)

#### PID 0x00 - PIDs Supported [01-20]
- **Description**: Bitmask showing which PIDs 0x01-0x20 are supported
- **Response Format**: 4 bytes (32 bits)
- **Supported PIDs**: 01, 03, 04, 05, 06, 07, 08, 09, 0B, 0C, 0D, 0E, 0F, 10, 11, 13, 14, 15, 19, 1C, 1F, 20
- **Bitmask**: `0xBFBEB893`

#### PID 0x20 - PIDs Supported [21-40]
- **Description**: Bitmask showing which PIDs 0x21-0x40 are supported
- **Response Format**: 4 bytes (32 bits)
- **Supported PIDs**: 21, 23, 2E, 2F, 30, 31, 32, 33, 34, 38, 3C, 3D, 40
- **Bitmask**: `0xA007F119`

#### PID 0x40 - PIDs Supported [41-60]
- **Description**: Bitmask showing which PIDs 0x41-0x60 are supported
- **Response Format**: 4 bytes (32 bits)
- **Supported PIDs**: 41, 42, 43, 44, 45, 46, 47, 49, 4A, 4C, 51, 56, 58
- **Bitmask**: `0xFED08500`

### Engine Performance PIDs

#### PID 0x01 - Monitor Status Since DTCs Cleared
- **Description**: MIL status and emissions monitor readiness
- **Formula**: Bit-encoded (see SAE J1979 for full breakdown)
  - Byte A, Bit 7: MIL status (0=off, 1=on)
  - Byte A, Bits 0-6: Number of DTCs stored
  - Bytes B-D: Monitor completion status
- **Data Range**: 4 bytes
- **Example**: `0x00 07 E5 00` = MIL off, 0 DTCs, monitors complete
- **Emissions Relevance**: Shows which emission monitors have completed their self-tests

#### PID 0x04 - Calculated Engine Load
- **Description**: Current engine load percentage (affects emissions strategy)
- **Formula**: `A × 100/255` (%)
- **Data Range**: 0-100%
- **Example Values**:
  - Idle: `0x3D` → 24.3%
  - City: `0x50` → 31.4%
  - Highway: `0x60` → 37.6%
  - Accelerating: `0x80` → 50.2%
- **Emissions Relevance**: ECU uses load to determine fuel enrichment and spark timing

#### PID 0x05 - Engine Coolant Temperature
- **Description**: Coolant temperature (critical for cold-start emissions)
- **Formula**: `A - 40` (°C)
- **Data Range**: -40°C to 215°C
- **Example**: `0x87` → 95°C (optimal for catalyst operation)
- **Emissions Relevance**: Cold engines produce 10x more emissions until reaching 95°C

#### PID 0x0B - Intake Manifold Absolute Pressure
- **Description**: Manifold pressure for load calculation
- **Formula**: `A` (kPa)
- **Data Range**: 0-255 kPa
- **Example**: `0x21` → 33 kPa (idle vacuum)
- **Emissions Relevance**: Used with MAF for fuel delivery calculations

#### PID 0x0C - Engine RPM
- **Description**: Current engine speed
- **Formula**: `((A × 256) + B) / 4` (RPM)
- **Data Range**: 0-16,383.75 RPM
- **Example Values**:
  - Idle: `0x0990` → 612 RPM
  - City: `0x0FA0` → 1,000 RPM
  - Highway: `0x1900` → 1,600 RPM
  - Accelerating: `0x1C20` → 1,800 RPM
- **Emissions Relevance**: RPM affects catalyst light-off and fuel delivery

#### PID 0x0D - Vehicle Speed
- **Description**: Current vehicle speed
- **Formula**: `A` (km/h)
- **Data Range**: 0-255 km/h
- **Example Values**:
  - Idle: `0x00` → 0 km/h
  - City: `0x2D` → 45 km/h
  - Highway: `0x4E` → 78 km/h
- **Emissions Relevance**: Speed determines drive cycle phase for monitor completion

#### PID 0x0E - Timing Advance
- **Description**: Spark timing advance before TDC
- **Formula**: `A/2 - 64` (degrees)
- **Data Range**: -64° to 63.5°
- **Example**: `0x8C` → 6.0° advance
- **Emissions Relevance**: Timing affects NOx emissions and combustion efficiency

#### PID 0x0F - Intake Air Temperature
- **Description**: Air temperature entering engine
- **Formula**: `A - 40` (°C)
- **Data Range**: -40°C to 215°C
- **Example**: `0x65` → 61°C
- **Emissions Relevance**: Cold air increases NOx, hot air reduces power and increases HC

#### PID 0x10 - MAF Air Flow Rate
- **Description**: Mass air flow sensor reading
- **Formula**: `((A × 256) + B) / 100` (g/s)
- **Data Range**: 0-655.35 g/s
- **Example**: Scales dynamically with RPM (2-25 g/s typical)
- **Emissions Relevance**: Primary input for fuel delivery calculation

#### PID 0x11 - Throttle Position
- **Description**: Absolute throttle position
- **Formula**: `A × 100/255` (%)
- **Data Range**: 0-100%
- **Example Values**:
  - Idle: `0x1E` → 11.8%
  - City: `0x40` → 25.1%
  - Highway: `0x4A` → 29.0%
  - Accelerating: `0x80` → 50.2%
- **Emissions Relevance**: Determines fuel enrichment and power output

#### PID 0x1F - Run Time Since Engine Start
- **Description**: Time engine has been running
- **Formula**: `(A × 256) + B` (seconds)
- **Data Range**: 0-65,535 seconds (~18 hours)
- **Example**: `0x2AAE` → 10,926 seconds (3 hours 2 minutes)
- **Emissions Relevance**: Required for catalyst warm-up monitoring

### Fuel System PIDs

#### PID 0x03 - Fuel System Status
- **Description**: Current fuel system operating mode
- **Formula**: Bit-encoded
  - `0x01` = Open loop (insufficient temp)
  - `0x02` = Closed loop (using O2 sensor)
  - `0x04` = Open loop (engine load)
  - `0x08` = Open loop (system fault)
- **Example**: `0x02 00` → Closed loop operation
- **Emissions Relevance**: Closed loop required for emissions compliance

#### PID 0x06 - Short Term Fuel Trim - Bank 1
- **Description**: Immediate fuel correction based on O2 sensor
- **Formula**: `(A - 128) × 100/128` (%)
- **Data Range**: -100% to +99.2%
- **Example**: `0x7F` → -0.8% (slightly lean correction)
- **Emissions Relevance**: Should oscillate ±5% around stoichiometric in closed loop

#### PID 0x07 - Long Term Fuel Trim - Bank 1
- **Description**: Learned fuel correction over time
- **Formula**: `(A - 128) × 100/128` (%)
- **Data Range**: -100% to +99.2%
- **Example**: `0x83` → +2.3% (learned rich correction)
- **Emissions Relevance**: Values >±10% indicate fuel system or sensor problems

#### PID 0x08 - Short Term Fuel Trim - Bank 2
- **Description**: Immediate fuel correction for bank 2
- **Formula**: `(A - 128) × 100/128` (%)
- **Data Range**: -100% to +99.2%
- **Example**: `0x7F` → -0.8%
- **Emissions Relevance**: Should match Bank 1 trim within ±5%

#### PID 0x09 - Long Term Fuel Trim - Bank 2
- **Description**: Learned fuel correction for bank 2
- **Formula**: `(A - 128) × 100/128` (%)
- **Data Range**: -100% to +99.2%
- **Example**: `0x7B` → -3.9%
- **Emissions Relevance**: Excessive difference between banks indicates issues

#### PID 0x23 - Fuel Rail Pressure (Gauge)
- **Description**: Fuel pressure in direct injection systems
- **Formula**: `((A × 256) + B) × 10` (kPa)
- **Data Range**: 0-655,350 kPa
- **Example**: `0x0028` → 400 kPa (typical GDI pressure)
- **Emissions Relevance**: Proper atomization reduces particulate emissions

#### PID 0x2F - Fuel Tank Level Input
- **Description**: Fuel level percentage
- **Formula**: `A × 100/255` (%)
- **Data Range**: 0-100%
- **Example**: `0x39` → 22.4%
- **Emissions Relevance**: Used for EVAP system leak detection

#### PID 0x51 - Fuel Type
- **Description**: Type of fuel used by vehicle
- **Formula**: Enumerated value
  - `0x01` = Gasoline
  - `0x02` = Methanol
  - `0x03` = Ethanol
  - `0x04` = Diesel
  - (See SAE J1979 for complete list)
- **Example**: `0x01` → Gasoline
- **Emissions Relevance**: Determines stoichiometric ratio and emission standards

### Oxygen Sensor PIDs

#### PID 0x13 - Oxygen Sensors Present
- **Description**: Bitmask showing which O2 sensors are present
- **Formula**: Bit-encoded (2 banks × 4 sensors)
- **Example**: `0x33` → Bank 1: Sensors 1,2; Bank 2: Sensors 1,2
- **Emissions Relevance**: Identifies active sensors for emissions feedback

#### PID 0x14 - Oxygen Sensor 1, Bank 1 (Voltage)
- **Description**: Primary O2 sensor voltage (upstream of catalyst)
- **Formula**: `A × 0.005` (V), `B × 100/128 - 100` (% STFT if applicable)
- **Data Range**: 0-1.275V
- **Example**: `0x46` → 0.35V (lean), `0x6E` → 0.55V (rich)
- **Emissions Relevance**: Should oscillate 0.1-0.9V in closed loop, centered at 0.45V

#### PID 0x15 - Oxygen Sensor 2, Bank 1 (Voltage)
- **Description**: Secondary O2 sensor (downstream of catalyst)
- **Formula**: `A × 0.005` (V)
- **Data Range**: 0-1.275V
- **Example**: Dynamic oscillation between 0.35V-0.55V
- **Emissions Relevance**: Flat signal indicates efficient catalyst, oscillation indicates failure

#### PID 0x19 - Oxygen Sensor 2, Bank 2 (Voltage)
- **Description**: Secondary O2 sensor for bank 2
- **Formula**: `A × 0.005` (V)
- **Data Range**: 0-1.275V
- **Example**: Similar to bank 1 with slight variation
- **Emissions Relevance**: Monitors catalyst efficiency on second bank

#### PID 0x34 - Oxygen Sensor 1, Bank 1 (Wide Range)
- **Description**: Wide-range O2 sensor (Air-Fuel Equivalence Ratio)
- **Formula**:
  - Equivalence Ratio: `((A × 256) + B) × 2/65536`
  - Voltage: `((C × 256) + D) × 8/65536` (V)
- **Data Range**: 0-2 (ratio), 0-8V
- **Example**: `0x80A7 8000` → Ratio ≈1.0 (stoichiometric), Voltage ≈4.0V
- **Emissions Relevance**: More precise than narrow-band sensors for lean-burn engines

#### PID 0x38 - Oxygen Sensor 5, Bank 2 (Wide Range)
- **Description**: Wide-range O2 sensor for bank 2
- **Formula**: Same as PID 0x34
- **Example**: `0x8037 7FFD`
- **Emissions Relevance**: Cross-bank comparison for diagnosis

#### PID 0x56 - Short Term Secondary O2 Sensor Trim - Bank 1
- **Description**: Secondary O2 trim correction
- **Formula**: `(A - 128) × 100/128` (%)
- **Data Range**: -100% to +99.2%
- **Example**: `0x7E` → -1.6%
- **Emissions Relevance**: Post-catalyst O2 feedback for advanced systems

#### PID 0x58 - Short Term Secondary O2 Sensor Trim - Bank 2
- **Description**: Secondary O2 trim for bank 2
- **Formula**: `(A - 128) × 100/128` (%)
- **Data Range**: -100% to +99.2%
- **Example**: `0x7F` → -0.8%
- **Emissions Relevance**: Ensures both banks maintain emissions compliance

### Catalyst System PIDs

#### PID 0x3C - Catalyst Temperature - Bank 1, Sensor 1
- **Description**: Catalyst temperature monitoring
- **Formula**: `((A × 256) + B)/10 - 40` (°C)
- **Data Range**: -40°C to 6,513.5°C
- **Example**: `0x117F` → 407.5°C (normal operating temp)
- **Emissions Relevance**: Catalyst operates 400-800°C; below 250°C is ineffective

#### PID 0x3D - Catalyst Temperature - Bank 2, Sensor 1
- **Description**: Catalyst temperature for bank 2
- **Formula**: `((A × 256) + B)/10 - 40` (°C)
- **Data Range**: -40°C to 6,513.5°C
- **Example**: `0x117E` → 407.4°C
- **Emissions Relevance**: Should match bank 1 within ±50°C

### EVAP System PIDs

#### PID 0x2E - Commanded Evaporative Purge
- **Description**: EVAP purge valve duty cycle
- **Formula**: `A × 100/255` (%)
- **Data Range**: 0-100%
- **Example**: `0x79` → 47.5% (active purging)
- **Emissions Relevance**: Purges fuel vapors into intake to reduce HC emissions

#### PID 0x32 - Evap System Vapor Pressure
- **Description**: EVAP system pressure for leak detection
- **Formula**: `((A × 256) + B) - 32768` (Pa)
- **Data Range**: -32,768 to 32,767 Pa
- **Example**: `0xFDDD` → -547 Pa (slight vacuum)
- **Emissions Relevance**: Leak detection required for emissions compliance

### Environmental PIDs

#### PID 0x33 - Absolute Barometric Pressure
- **Description**: Ambient atmospheric pressure
- **Formula**: `A` (kPa)
- **Data Range**: 0-255 kPa
- **Example**: `0x62` → 98 kPa (sea level ≈101 kPa)
- **Emissions Relevance**: Altitude compensation for fuel delivery

#### PID 0x46 - Ambient Air Temperature
- **Description**: Outside air temperature
- **Formula**: `A - 40` (°C)
- **Data Range**: -40°C to 215°C
- **Example**: `0x4E` → 38°C
- **Emissions Relevance**: Affects cold start enrichment and EVAP testing

### Diagnostic and Status PIDs

#### PID 0x1C - OBD Standards
- **Description**: OBD standard vehicle conforms to
- **Formula**: Enumerated value
  - `0x01` = OBD-II (CARB)
  - `0x02` = OBD (EPA)
  - `0x03` = OBD and OBD-II
  - `0x04` = OBD-I
  - (See SAE J1979 for complete list)
- **Example**: `0x03` → OBD and OBD-II
- **Emissions Relevance**: Determines applicable emission standards

#### PID 0x21 - Distance Traveled with MIL On
- **Description**: Distance driven since MIL illuminated
- **Formula**: `(A × 256) + B` (km)
- **Data Range**: 0-65,535 km
- **Example**: `0x0000` → 0 km (no active MIL)
- **Emissions Relevance**: I/M programs check if MIL was recently cleared

#### PID 0x30 - Warm-ups Since DTCs Cleared
- **Description**: Count of engine warm-up cycles
- **Formula**: `A` (count)
- **Data Range**: 0-255
- **Example**: `0xFF` → 255 warm-ups
- **Emissions Relevance**: Monitors must complete within 40 warm-up cycles

#### PID 0x31 - Distance Since DTCs Cleared
- **Description**: Distance since codes were cleared
- **Formula**: `(A × 256) + B` (km)
- **Data Range**: 0-65,535 km
- **Example**: `0xFFFF` → 65,535 km
- **Emissions Relevance**: Indicates if vehicle was recently cleared before I/M test

#### PID 0x41 - Monitor Status This Drive Cycle
- **Description**: Monitor completion status for current cycle
- **Formula**: Bit-encoded (similar to PID 0x01)
- **Data Range**: 4 bytes
- **Example**: `0x00 05 E0 24`
- **Emissions Relevance**: Shows real-time monitor completion progress

#### PID 0x42 - Control Module Voltage
- **Description**: Battery/system voltage
- **Formula**: `((A × 256) + B) / 1000` (V)
- **Data Range**: 0-65.535V
- **Example**: `0x33FF` → 13.31V (normal charging system)
- **Emissions Relevance**: Low voltage can affect sensor readings and emissions

#### PID 0x43 - Absolute Load Value
- **Description**: Normalized engine load
- **Formula**: `((A × 256) + B) × 100/255` (%)
- **Data Range**: 0-25,700%
- **Example**: `0x002D` → 17.6%
- **Emissions Relevance**: More accurate load calculation for emissions strategies

### Advanced Throttle PIDs

#### PID 0x44 - Commanded Equivalence Ratio
- **Description**: Target air-fuel ratio
- **Formula**: `((A × 256) + B) × 2/65536`
- **Data Range**: 0-2
- **Example**: `0x7FFF` → 1.0 (stoichiometric)
- **Emissions Relevance**: Target ratio for catalyst efficiency

#### PID 0x45 - Relative Throttle Position
- **Description**: Relative throttle opening
- **Formula**: `A × 100/255` (%)
- **Data Range**: 0-100%
- **Example**: Calculated as 1/4 of absolute throttle
- **Emissions Relevance**: Used for decel fuel cutoff (DFCO) strategy

#### PID 0x47 - Absolute Throttle Position B
- **Description**: Secondary throttle position sensor
- **Formula**: `A × 100/255` (%)
- **Data Range**: 0-100%
- **Example**: Matches PID 0x11 in this implementation
- **Emissions Relevance**: Redundant sensor for drive-by-wire systems

#### PID 0x49 - Accelerator Pedal Position D
- **Description**: Pedal position sensor reading
- **Formula**: `A × 100/255` (%)
- **Data Range**: 0-100%
- **Example**: `0x11` → 6.7%
- **Emissions Relevance**: Determines driver demand for emissions strategy

#### PID 0x4A - Accelerator Pedal Position E
- **Description**: Secondary pedal position sensor
- **Formula**: `A × 100/255` (%)
- **Data Range**: 0-100%
- **Example**: `0x11` → 6.7%
- **Emissions Relevance**: Redundancy for electronic throttle control

#### PID 0x4C - Commanded Throttle Actuator
- **Description**: ECU's commanded throttle position
- **Formula**: `A × 100/255` (%)
- **Data Range**: 0-100%
- **Example**: Half of actual throttle position
- **Emissions Relevance**: Shows how ECU modifies driver input for emissions

## Recent Fixes and Enhancements

### Version 3.0 - Modular Architecture (Current)
- **Refactored to plugin-based architecture**: Mode handlers now auto-register at compile time
- **Added proper negative responses**: Unsupported PIDs return ISO 14229 compliant 0x7F response
- **Removed duplicate handlers**: Consolidated Mode 01 implementation

### CAN ID Communication Fix (Commit 300f14c)
- **Issue**: 20 PIDs were missing `PID_REPLY_ENGINE` (0x7E8) CAN ID assignments
- **Fixed PIDs**: 0x08, 0x09, 0x0B, 0x0E, 0x0F, 0x13, 0x15, 0x19, 0x1C, 0x1F, 0x21, 0x23, 0x2E, 0x2F, 0x30
- **Impact**: Scanners could not detect or communicate with these PIDs
- **Resolution**: All Mode 01 responses now properly use 0x7E8 CAN ID

### O2 Voltage Formula Enhancement
- **Improved PID 0x14 implementation**: Now includes proper voltage calculation
- **Formula**: `A × 0.005V` per SAE J1979 standard
- **Dynamic Range**: 0.35V-0.55V oscillation simulating real O2 sensor
- **Behavior**: Realistic rich/lean cycling around stoichiometric (0.45V center)

### Fuel Pressure Accuracy (PID 0x23)
- **Corrected Formula**: `((A × 256) + B) × 10 kPa`
- **Target Pressure**: 400 kPa (typical for gasoline direct injection)
- **Value**: `0x0028` = 40 decimal × 10 = 400 kPa
- **Relevance**: Matches real-world GDI fuel system pressures

### PID 0x14 Addition
- **Added**: O2 Sensor 1 Bank 1 voltage reading
- **Bitmask Update**: PID 0x00 now includes PID 0x14 in supported list
- **Implementation**: Byte 3 contains voltage, Byte 4 set to 0xFF (STFT not used in this format)

## Multi-ECU Response Behavior

The simulator accurately reproduces multi-ECU behavior found in real vehicles:

### ECU Response Strategy

1. **Engine ECU (0x7E8)**: Responds to all emissions-related PIDs
   - Primary powertrain data source
   - Handles engine, fuel, and emissions sensors
   - Responds to all PID support queries

2. **Transmission ECU (0x7E9)**: Limited PID support
   - Only responds to PID support queries (0x00, 0x20, 0x40)
   - Returns zeros for unsupported PIDs
   - Simulates realistic scan tool detection behavior

### Response Timing
- **Engine ECU**: Immediate response
- **Transmission ECU**: 5ms delay after engine response
- **Purpose**: Allows scan tools to detect multiple ECU presence

### Broadcast Handling
- **Request CAN ID**: 0x7DF (functional/broadcast)
- **Multiple Responses**: Both ECUs respond to PID support requests
- **Scanner Detection**: Professional tools identify number of ECUs present

## Dynamic Driving Simulation States

The simulator cycles through five realistic driving states every 10 seconds with smooth value transitions:

### 1. IDLE State
**Characteristics:**
- **RPM**: 600-650 (oscillating around 612 RPM baseline)
- **Speed**: 0 km/h
- **Load**: ~24% (0x3D ± variation)
- **Throttle**: 11.8% (0x1E)
- **MAF**: Minimal airflow (~2 g/s)
- **O2 Voltage**: 0.35-0.55V oscillating

**Emissions Behavior:**
- Closed-loop fuel control active
- Minimal NOx and HC production
- Catalyst at optimal temperature (95°C)

### 2. CITY State
**Characteristics:**
- **RPM**: 1000-1500 (variable)
- **Speed**: 15-50 km/h
- **Load**: ~35% (0x50 ± variation)
- **Throttle**: 25% (0x40)
- **MAF**: 8-12 g/s
- **O2 Voltage**: Active oscillation

**Emissions Behavior:**
- EPA FTP-75 urban cycle simulation
- Moderate NOx production
- Fuel trims actively adjusting

### 3. ACCELERATING State
**Characteristics:**
- **RPM**: 1800-2500 (increasing)
- **Speed**: Ramping up to 80 km/h
- **Load**: ~50% (0x80 ± variation)
- **Throttle**: 50% (0x80)
- **MAF**: 18-22 g/s
- **O2 Voltage**: Momentary rich bias

**Emissions Behavior:**
- Power enrichment phase
- Higher emissions temporarily
- May enter open-loop briefly

### 4. HIGHWAY State
**Characteristics:**
- **RPM**: 1600 (steady, from Mercedes data)
- **Speed**: 78-79 km/h (cruise)
- **Load**: ~38% (0x60 ± variation)
- **Throttle**: 29% (0x4A)
- **MAF**: 14-16 g/s
- **O2 Voltage**: Stable oscillation

**Emissions Behavior:**
- HWFET cycle conditions
- Optimal fuel efficiency
- Lowest emissions per km
- Lean cruise operation

### 5. BRAKING State
**Characteristics:**
- **RPM**: Decreasing (down to idle)
- **Speed**: Decreasing (down to 0)
- **Load**: Low (~12%, 0x20)
- **Throttle**: 0% (0x00)
- **MAF**: Minimal
- **O2 Voltage**: >0.8V (lean)

**Emissions Behavior:**
- Decel Fuel Cut-Off (DFCO) active
- Zero fuel consumption
- O2 shows lean condition
- Engine braking mode

### Value Update Mechanism
- **State Change**: Every 10 seconds (randomized selection)
- **Value Updates**: Every 100ms for smooth transitions
- **Correlations**: RPM/Speed/Load/Throttle properly linked
- **Randomization**: ± variation to simulate real sensor noise
- **O2 Oscillation**: Continuous rich/lean cycling at 0.35-0.55V

## Protocol Compliance

### ISO 15765-4 (CAN) Compliance
- **Single Frame Format**: `[PCI:1] [Mode+0x40:1] [PID:1] [Data:0-5] [Padding:0-4]`
- **Response Mode**: Request Mode + 0x40 (e.g., 0x01 → 0x41)
- **CAN ID**: 0x7E8 for all engine responses
- **Message Length**: 8 bytes total
- **PCI Byte**: `0x0N` where N = number of data bytes + 2

### SAE J1979 Compliance
- **Formula Accuracy**: All formulas match SAE J1979 specification
- **Data Ranges**: Values within specified limits
- **Bit Encoding**: Proper bitmask implementation for PID support
- **Negative Response**: 0x7F response for unsupported PIDs (NRC 0x12)

### ISO 14229 (UDS) Extensions
- **Negative Response Format**: `7F [Service] [PID] [NRC]`
- **NRC 0x12**: Sub-function Not Supported (requestSequenceError)
- **Proper Error Handling**: Graceful rejection of invalid requests

## Usage Examples

### Example 1: Reading Engine RPM (PID 0x0C)

**Request:**
```
CAN ID: 0x7DF (broadcast)
Data: [02 01 0C 00 00 00 00 00]
      [Len Mode PID Padding      ]
```

**Response:**
```
CAN ID: 0x7E8 (engine ECU)
Data: [04 41 0C 09 90 00 00 00]
      [Len +40 PID  A  B Padding]
```

**Calculation:**
```
RPM = ((A × 256) + B) / 4
    = ((0x09 × 256) + 0x90) / 4
    = (2304 + 144) / 4
    = 2448 / 4
    = 612 RPM (idle)
```

### Example 2: Reading O2 Sensor Voltage (PID 0x14)

**Request:**
```
CAN ID: 0x7DF
Data: [02 01 14 00 00 00 00 00]
```

**Response:**
```
CAN ID: 0x7E8
Data: [04 41 14 46 FF 00 00 00]
      [Len +40 PID  A  B Pad   ]
```

**Calculation:**
```
Voltage = A × 0.005 V
        = 0x46 × 0.005
        = 70 × 0.005
        = 0.35V (lean condition)
```

### Example 3: Checking Supported PIDs (PID 0x00)

**Request:**
```
CAN ID: 0x7DF
Data: [02 01 00 00 00 00 00 00]
```

**Response 1 (Engine ECU):**
```
CAN ID: 0x7E8
Data: [06 41 00 BF BE B8 93 00]
      [Len +40 PID Bitmask   Pad]
```

**Response 2 (Transmission ECU, 5ms later):**
```
CAN ID: 0x7E9
Data: [06 41 00 18 00 00 00 00]
```

**Decoding Engine Bitmask:**
```
0xBF = 1011 1111 → PIDs 01,03,04,05,06,07,08,09
0xBE = 1011 1110 → PIDs 0B,0C,0D,0E,0F,10
0xB8 = 1011 1000 → PIDs 11,13,14,15
0x93 = 1001 0011 → PIDs 19,1C,1F,20
```

### Example 4: Reading Fuel Trim (PID 0x06)

**Request:**
```
CAN ID: 0x7DF
Data: [02 01 06 00 00 00 00 00]
```

**Response:**
```
CAN ID: 0x7E8
Data: [03 41 06 7F 00 00 00 00]
```

**Calculation:**
```
STFT = (A - 128) × 100/128 %
     = (0x7F - 128) × 100/128
     = (127 - 128) × 100/128
     = -1 × 100/128
     = -0.78% (slightly lean correction)
```

### Example 5: Unsupported PID Response (PID 0xFF)

**Request:**
```
CAN ID: 0x7DF
Data: [02 01 FF 00 00 00 00 00]
```

**Response (Negative):**
```
CAN ID: 0x7E8
Data: [03 7F 01 FF 12 00 00 00]
      [Len NR Svc PID NRC Pad   ]
```

**Meaning:**
- `7F` = Negative Response Service Identifier
- `01` = Echo requested service (Mode 01)
- `FF` = Echo requested PID
- `12` = NRC: Sub-function Not Supported

## Integration with Other Modes

Mode 01 data integrates with other OBD-II modes:

### Mode 02 (Freeze Frame)
- When DTC triggers, current Mode 01 values are captured
- Stored freeze frame contains: RPM, Speed, Load, Coolant Temp, Throttle, MAF, O2 Voltage
- Retrieved via Mode 02 with same PID numbers

### Mode 03 (Stored DTCs)
- PID 0x01 Monitor Status shows DTC count
- MIL status in PID 0x01 indicates if DTCs are present

### Mode 04 (Clear DTCs)
- Resets PID 0x21 (Distance with MIL) to 0
- Resets PID 0x30 (Warm-ups) to 0
- Resets PID 0x31 (Distance since clear) to 0
- Clears Monitor Status (PID 0x01)

### Mode 09 (Vehicle Info)
- ECU identification determines which Mode 01 PIDs are supported
- Performance tracking (Mode 09 PID 0x08) uses Mode 01 monitor data

## Data Source and Validation

All Mode 01 values are based on **real Mercedes-Benz vehicle data**:

- **Vehicle**: Mercedes-Benz (VIN: 4JGDA5HB7JB158144)
- **Data Points**: 7,510+ logged OBD-II responses
- **Capture Method**: Professional scan tool with 1Hz sampling
- **Driving Conditions**: Idle, City, Highway, Acceleration, Braking
- **Validation**: Cross-referenced with SAE J1979 formulas
- **Temperature**: All data captured at normal operating temperature (95°C)

### Real-World Values Used
- **Idle**: 612 RPM @ 0 km/h, 95°C coolant
- **Highway**: 1600 RPM @ 78 km/h, load 38%
- **Fuel Pressure**: 400 kPa (GDI system)
- **O2 Sensors**: 0.35-0.55V oscillation (closed loop)
- **Battery Voltage**: 13.31V (charging)
- **Catalyst Temp**: 407°C (normal operation)

## Technical Notes for Developers

### Adding New PIDs
1. Add PID definition to `ecu_sim.h`
2. Update PID support bitmask in `handle_mode_01()`
3. Add case statement with proper formula
4. Set correct `can_MsgTx.id = PID_REPLY_ENGINE`
5. Update this documentation

### Testing Checklist
- [ ] Verify CAN ID is 0x7E8 (engine) or 0x7E9 (trans)
- [ ] Confirm response format: `[Len] [41] [PID] [Data...]`
- [ ] Validate formula against SAE J1979
- [ ] Test with professional scan tool
- [ ] Verify realistic value ranges
- [ ] Check multi-ECU response timing

### Common Pitfalls
1. **Missing CAN ID**: Always set `can_MsgTx.id` before `can1.write()`
2. **Wrong Response Mode**: Mode 01 response is 0x41, not 0x01
3. **Incorrect Formula**: Double-check SAE J1979 specification
4. **Buffer Overflow**: Mode 01 is single-frame, max 5 data bytes
5. **Static Values**: Use dynamic simulation, not fixed constants

## References

- **SAE J1979**: E/E Diagnostic Test Modes (Official OBD-II Standard)
- **ISO 15765-4**: Diagnostic communication over Controller Area Network (CAN)
- **ISO 14229-1**: Unified Diagnostic Services (UDS)
- **EPA Regulations**: 40 CFR Part 86 (Emissions Standards)
- **CARB Requirements**: California Code of Regulations, Title 13

---

**Document Version**: 1.0
**Last Updated**: 2025-10-03
**Maintainer**: Waleed Judah (Wal33D)
**Email**: aquataze@yahoo.com
