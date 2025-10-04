# OBD-II Mode 02 - Freeze Frame Data

## Overview

Mode 02 provides access to **freeze frame data** - a historical snapshot of critical emissions-related sensor values captured at the exact moment a Diagnostic Trouble Code (DTC) was triggered. This mode is mandated by OBD-II regulations (SAE J1979, ISO 15031-5) to help automotive technicians diagnose intermittent emissions failures.

## Purpose in OBD-II Emissions Monitoring

### Why Freeze Frames Matter

Freeze frame data is essential for emissions diagnosis because:

- **Captures Fault Conditions**: Records the exact operating conditions when an emissions-related fault occurred
- **Diagnoses Intermittent Issues**: Helps identify problems that may not be present during shop inspection
- **Historical Context**: Provides technicians with critical data about when and how the fault happened
- **Regulatory Compliance**: Required by EPA/CARB for all emissions-related DTCs

### Real-World Example

If a vehicle has a misfire DTC (P0300) that only occurs under specific conditions, freeze frame data reveals:
- Was the engine cold or warm when it misfired?
- What was the throttle position and load?
- What speed was the vehicle traveling?
- What were the O2 sensor readings?

This historical context is invaluable for reproducing and fixing the fault.

## Freeze Frame Concept

### When Data is Captured

The ECU automatically captures a freeze frame snapshot when:

1. **Emissions-Related DTC is Set**: A fault is detected that affects emissions compliance
2. **Fault Conditions Met**: The diagnostic test has failed according to OBD-II criteria
3. **MIL Illumination**: The Check Engine Light is triggered (for Type A DTCs)

### What Gets Captured

Each freeze frame stores a complete snapshot of:

| Parameter | Description | Purpose |
|-----------|-------------|---------|
| **Engine RPM** | Engine speed at fault time | Indicates load and operating conditions |
| **Vehicle Speed** | Speed when fault occurred | Shows if stationary, city, or highway |
| **Coolant Temperature** | Engine temperature | Affects cold-start emissions and catalyst |
| **Throttle Position** | Throttle opening percentage | Indicates acceleration or deceleration |
| **Mass Air Flow (MAF)** | Air intake rate | Critical for fuel mixture calculations |
| **O2 Sensor Voltage** | Oxygen sensor reading | Primary emissions feedback signal |

### Data Persistence

Freeze frame data is:
- **Stored Permanently**: Remains in ECU memory until explicitly cleared
- **Survives Power Loss**: Persists through battery disconnection
- **Cleared by Mode 04**: Only removed when technician clears DTCs
- **Associated with DTC**: Each freeze frame links to a specific fault code

## Supported PIDs in Freeze Frames

The simulator supports the following PIDs for freeze frame retrieval:

### PID 0x00 - Supported PIDs [01-20]
```
Response: 42 00 E8 19 30 12
Indicates which PIDs are available in freeze frames
```

### PID 0x05 - Engine Coolant Temperature
```
Request:  02 05 [frame_num]
Response: 42 05 [temp]
Formula:  Temperature (°C) = Value - 40
Range:    -40°C to 215°C
```

### PID 0x0C - Engine RPM
```
Request:  02 0C [frame_num]
Response: 42 0C [A] [B]
Formula:  RPM = (A × 256 + B) / 4
Range:    0 to 16,383.75 RPM
```

### PID 0x0D - Vehicle Speed
```
Request:  02 0D [frame_num]
Response: 42 0D [speed]
Formula:  Speed (km/h) = Value
Range:    0 to 255 km/h
```

### PID 0x10 - Mass Air Flow (MAF)
```
Request:  02 10 [frame_num]
Response: 42 10 [A] [B]
Formula:  MAF (grams/sec) = (A × 256 + B) / 100
Range:    0 to 655.35 g/s
```

### PID 0x11 - Throttle Position
```
Request:  02 11 [frame_num]
Response: 42 11 [position]
Formula:  Throttle (%) = (Value × 100) / 255
Range:    0% to 100%
```

### PID 0x14 - O2 Sensor Voltage (Bank 1, Sensor 1)
```
Request:  02 14 [frame_num]
Response: 42 14 [voltage] [trim]
Formula:  Voltage = Value × 0.005V
Range:    0 to 1.275V
```

## Simulator Storage: 2 Independent Freeze Frames

This simulator stores **2 independent freeze frames** to support multiple DTCs:

### Frame 0 - Primary DTC
- **Associated DTC**: P0100 (Mass Air Flow Circuit Malfunction)
- **Frame Number**: 0x00
- **Storage Index**: `freeze_frame[0]`
- **Triggered By**: First DTC set (pushbutton SW1)

### Frame 1 - Secondary DTC
- **Associated DTC**: P0200 (Injector Circuit Malfunction)
- **Frame Number**: 0x01
- **Storage Index**: `freeze_frame[1]`
- **Triggered By**: First DTC set (same trigger as Frame 0)

### Storage Structure

```cpp
typedef struct {
    unsigned char coolant_temp;      // Engine temp when fault occurred
    unsigned int engine_rpm;          // RPM at time of fault
    unsigned char throttle_position;  // Throttle position during fault
    unsigned char vehicle_speed;      // Speed when fault detected
    unsigned int maf_airflow;         // Airflow reading at fault time
    unsigned int o2_voltage;          // O2 voltage when DTC set
    bool data_stored;                 // Valid data flag
    unsigned int dtc_code;            // DTC that triggered capture (P0100/P0200)
} freeze_frame_t;

extern freeze_frame_t freeze_frame[2];  // Global storage for 2 frames
```

### Data Capture Logic

When a DTC is triggered (button press or fault condition):

```cpp
// Capture freeze frame for P0100
freeze_frame[0].coolant_temp = ecu.coolant_temp;
freeze_frame[0].engine_rpm = ecu.engine_rpm;
freeze_frame[0].throttle_position = ecu.throttle_position;
freeze_frame[0].vehicle_speed = ecu.vehicle_speed;
freeze_frame[0].maf_airflow = ecu.maf_airflow;
freeze_frame[0].o2_voltage = ecu.o2_voltage;
freeze_frame[0].data_stored = true;
freeze_frame[0].dtc_code = 0x0100;  // P0100

// Simultaneously capture freeze frame for P0200
freeze_frame[1].coolant_temp = ecu.coolant_temp;
freeze_frame[1].engine_rpm = ecu.engine_rpm;
freeze_frame[1].throttle_position = ecu.throttle_position;
freeze_frame[1].vehicle_speed = ecu.vehicle_speed;
freeze_frame[1].maf_airflow = ecu.maf_airflow;
freeze_frame[1].o2_voltage = ecu.o2_voltage;
freeze_frame[1].data_stored = true;
freeze_frame[1].dtc_code = 0x0200;  // P0200
```

## DTC Association

### P0100 - Mass Air Flow (MAF) Circuit Malfunction
- **Frame Number**: 0x00
- **Emissions Impact**: Incorrect MAF readings cause improper fuel calculations
- **Symptoms**: Rich/lean conditions, poor fuel economy, failed emissions test
- **Freeze Frame Use**: Shows conditions when MAF signal was out of range

### P0200 - Injector Circuit Malfunction (Bank 1)
- **Frame Number**: 0x01
- **Emissions Impact**: Injector problems cause misfires and unburned fuel
- **Symptoms**: Rough idle, increased HC emissions, catalyst damage
- **Freeze Frame Use**: Reveals operating conditions during injector fault

### Retrieving DTC-Specific Freeze Frames

To retrieve freeze frame for a specific DTC:

1. **Use Mode 03** to identify which DTCs are stored
2. **Use Mode 02** with appropriate frame number:
   - Frame 0x00 for first/primary DTC (P0100)
   - Frame 0x01 for second DTC (P0200)

## Data Persistence Until Mode 04 Clear

### Persistence Characteristics

Freeze frame data:
- **Remains Stored**: Persists indefinitely until cleared
- **Survives Resets**: Not affected by ECU resets or power cycles
- **Fault Resolution**: Stays even if fault condition clears
- **Overwrites**: New DTCs may overwrite old freeze frames (implementation-specific)

### Clearing Freeze Frames

Freeze frames are cleared **ONLY** by:

#### Mode 04 - Clear Diagnostic Information
```
Request:  04
Response: 44
Result:   All freeze frames cleared
          freeze_frame[0].data_stored = false
          freeze_frame[1].data_stored = false
```

#### What Mode 04 Clears:
- All stored DTCs (P0100, P0200)
- All freeze frame data (Frame 0, Frame 1)
- MIL/Check Engine Light status
- DTC counters and flags

#### What Persists After Clear:
- Vehicle identification (VIN, Calibration IDs)
- Readiness monitor status (resets to "not ready")
- Current sensor data (continues updating)

### Why Persistence Matters

Freeze frames must persist because:
- Technician needs stable data for diagnosis
- Intermittent faults may take time to reproduce
- Legal/warranty documentation requires historical record
- Emissions testing may reference stored data

## Protocol Compliance

### SAE J1979 Standard Compliance

Mode 02 implementation follows SAE J1979 requirements:

- **Request Format**: `02 [PID] [Frame_Number]`
- **Response Format**: `42 [PID] [Data]`
- **Frame Numbering**: 0x00 for first DTC, 0x01 for second, etc.
- **No Data Response**: Returns empty frame if no freeze frame stored

### ISO 15031-5 Compliance

Follows ISO 15031-5 for emissions diagnostics:

- Freeze frames mandatory for emissions DTCs
- Same PID support as Mode 01 (current data)
- Historical snapshot at DTC set time
- Associated with specific fault codes

### CAN Protocol (ISO 15765-4)

Communication uses standard OBD-II CAN protocol:

- **Request ID**: 0x7DF (functional) or 0x7E0 (physical)
- **Response ID**: 0x7E8 (Engine ECU)
- **Baud Rate**: 500 kbps
- **Frame Format**: 11-bit identifier, 8-byte payload

### Single Frame Response

All Mode 02 responses fit in single CAN frame (≤7 data bytes):

```
Byte 0: [Length]     - Number of data bytes
Byte 1: 0x42         - Mode 02 response
Byte 2: [PID]        - Requested PID
Byte 3-7: [Data]     - PID-specific data
```

## Usage Examples with Actual Commands

### Example 1: Check for Freeze Frame Data

**Request supported PIDs in freeze frames:**
```
CAN ID: 0x7DF
Data:   02 02 00 00 00 00 00 00
        │  │  │
        │  │  └─ PID 00 (supported PIDs)
        │  └──── Mode 02 (freeze frame)
        └─────── Length (2 bytes)

Response:
CAN ID: 0x7E8
Data:   06 42 00 E8 19 30 12 00
        │  │  │  └──────────┘
        │  │  │       └─ Supported PID bitmap
        │  │  └───────── PID 00
        │  └──────────── Mode 02 response
        └─────────────── Length (6 bytes)
```

### Example 2: Retrieve Engine RPM from Freeze Frame 0

**Get RPM when P0100 (MAF fault) occurred:**
```
CAN ID: 0x7DF
Data:   03 02 0C 00 00 00 00 00
        │  │  │  │
        │  │  │  └─ Frame 0 (first DTC)
        │  │  └──── PID 0C (engine RPM)
        │  └─────── Mode 02
        └────────── Length (3 bytes)

Response:
CAN ID: 0x7E8
Data:   04 42 0C 09 9A 00 00 00
        │  │  │  └──┘
        │  │  │    └─ RPM data (0x099A = 2458)
        │  │  │       Actual RPM = 2458 / 4 = 614.5 RPM
        │  │  └────── PID 0C
        │  └───────── Mode 02 response
        └──────────── Length (4 bytes)
```

### Example 3: Retrieve Coolant Temperature from Freeze Frame 1

**Get coolant temp when P0200 (injector fault) occurred:**
```
CAN ID: 0x7DF
Data:   03 02 05 01 00 00 00 00
        │  │  │  │
        │  │  │  └─ Frame 1 (second DTC)
        │  │  └──── PID 05 (coolant temp)
        │  └─────── Mode 02
        └────────── Length (3 bytes)

Response:
CAN ID: 0x7E8
Data:   03 42 05 87 00 00 00 00
        │  │  │  │
        │  │  │  └─ Temp value (0x87 = 135)
        │  │  │     Actual temp = 135 - 40 = 95°C
        │  │  └──── PID 05
        │  └─────── Mode 02 response
        └────────── Length (3 bytes)
```

### Example 4: Retrieve Vehicle Speed at Fault Time

**Get speed when fault occurred (Frame 0):**
```
CAN ID: 0x7DF
Data:   03 02 0D 00 00 00 00 00
        │  │  │  │
        │  │  │  └─ Frame 0
        │  │  └──── PID 0D (vehicle speed)
        │  └─────── Mode 02
        └────────── Length (3 bytes)

Response:
CAN ID: 0x7E8
Data:   03 42 0D 00 00 00 00 00
        │  │  │  │
        │  │  │  └─ Speed = 0 km/h (vehicle was stopped)
        │  │  └──── PID 0D
        │  └─────── Mode 02 response
        └────────── Length (3 bytes)
```

### Example 5: Request Non-Existent Freeze Frame

**Request Frame 2 (only 0 and 1 exist):**
```
CAN ID: 0x7DF
Data:   03 02 0C 02 00 00 00 00
        │  │  │  │
        │  │  │  └─ Frame 2 (not stored)
        │  │  └──── PID 0C
        │  └─────── Mode 02
        └────────── Length (3 bytes)

Response:
CAN ID: 0x7E8
Data:   00 42 00 00 00 00 00 00
        │  │
        │  └─ Mode 02 response
        └──── Length 0 (no data available)
```

### Example 6: Complete Diagnostic Sequence

**Full workflow: Check DTC, retrieve freeze frame, clear:**

1. **Get stored DTCs (Mode 03):**
   ```
   Request:  01 03 00 00 00 00 00 00
   Response: 06 43 02 01 00 02 00 00
             │  │  │  └──┘  └──┘
             │  │  │    │     └─ DTC 2: P0200
             │  │  │    └─────── DTC 1: P0100
             │  │  └──────────── DTC count: 2
             │  └─────────────── Mode 03 response
             └────────────────── Length (6 bytes)
   ```

2. **Get freeze frame for P0100 (Frame 0):**
   ```
   Request:  03 02 0C 00 00 00 00 00  (RPM at fault)
   Response: 04 42 0C 09 9A 00 00 00  (614.5 RPM)
   ```

3. **Get freeze frame for P0200 (Frame 1):**
   ```
   Request:  03 02 05 01 00 00 00 00  (Coolant temp at fault)
   Response: 03 42 05 87 00 00 00 00  (95°C)
   ```

4. **Clear DTCs and freeze frames (Mode 04):**
   ```
   Request:  01 04 00 00 00 00 00 00
   Response: 01 44 00 00 00 00 00 00  (Cleared successfully)
   ```

5. **Verify freeze frames cleared:**
   ```
   Request:  03 02 0C 00 00 00 00 00
   Response: 00 42 00 00 00 00 00 00  (No data - cleared)
   ```

## Summary for Automotive Technicians

### Key Takeaways

1. **Mode 02 = Historical Data**: Unlike Mode 01 (live data), Mode 02 shows what happened when fault occurred

2. **Two Freeze Frames**: This simulator stores 2 frames for DTCs P0100 and P0200
   - Frame 0x00 → P0100 (MAF Circuit)
   - Frame 0x01 → P0200 (Injector Circuit)

3. **Captured Parameters**: RPM, speed, coolant temp, throttle, MAF, O2 voltage

4. **Data Persists**: Freeze frames remain until Mode 04 clear command

5. **Diagnostic Value**: Essential for troubleshooting intermittent emissions faults

### Diagnostic Workflow

```
1. Connect scan tool → Vehicle responds
2. Check DTCs (Mode 03) → Identifies P0100, P0200
3. Retrieve freeze frames (Mode 02) → Shows fault conditions
4. Analyze data → Determine root cause
5. Repair fault → Fix underlying issue
6. Clear codes (Mode 04) → Reset system
7. Test drive → Verify repair
8. Re-scan → Confirm no DTCs
```

### Common Scan Tool Commands

| Tool Function | OBD-II Mode | CAN Request | Response |
|---------------|-------------|-------------|----------|
| View DTCs | Mode 03 | `01 03` | `43 [count] [DTCs]` |
| Freeze Frame RPM | Mode 02 PID 0C | `03 02 0C [frame]` | `42 0C [RPM]` |
| Freeze Frame Speed | Mode 02 PID 0D | `03 02 0D [frame]` | `42 0D [speed]` |
| Clear All Data | Mode 04 | `01 04` | `44` (success) |

---

**Documentation Version**: 1.0
**Last Updated**: 2024
**Simulator Version**: v3.0 (Modular Architecture)
**Standards**: SAE J1979, ISO 15031-5, ISO 15765-4
